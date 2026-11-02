#! /usr/bin/env python3
# Copyright 2026 Neobotix GmbH
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Follow the configured node sequence only after an explicit start request."""

from copy import deepcopy

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputeAndTrackRoute
from nav2_simple_commander.robot_navigator import BasicNavigator, RunningTask, TaskResult
import rclpy
from rclpy.executors import ExternalShutdownException
from std_srvs.srv import Trigger


class RouteNavigator(BasicNavigator):
    def __init__(self):
        super().__init__(node_name='neo_route_node')
        self.declare_parameter('node_ids', [0])
        self._pending_node_ids = None
        self._running = False
        self._route_node_ids = None
        self._next_segment_index = 0
        self._resume_node_id = None
        self._route_request_token = None
        self._start_service = self.create_service(Trigger, '~/start', self._start)
        self._resume_service = self.create_service(Trigger, '~/resume', self._resume)
        self.info('Route following is idle. Set the initial pose in RViz, review node_ids, '
                  'then call ~/start.')

    def _start(self, request, response):
        if self._running:
            response.message = 'Route following is already running.'
            return response

        node_ids = list(self.get_parameter('node_ids').value)
        if len(node_ids) < 2 or any(node_id < 0 or node_id > 65535 for node_id in node_ids):
            response.message = 'node_ids must contain at least two IDs in [0, 65535].'
            return response
        if not (self.compute_and_track_route_client.server_is_ready()
                and self.compute_path_to_pose_client.server_is_ready()
                and self.follow_path_client.server_is_ready()):
            response.message = 'Nav2 route tracking, planner and controller action servers must be available.'
            return response

        # Start always replaces any saved progress with the current sequence.
        self._route_node_ids = node_ids
        self._next_segment_index = 0
        self._resume_node_id = None
        self._pending_node_ids = node_ids
        self._running = True
        response.success = True
        response.message = f'Route following accepted from the beginning for node IDs: {node_ids}'
        return response

    def _resume(self, request, response):
        if self._running:
            response.message = 'Route following is already running.'
            return response
        if (self._route_node_ids is None
                or self._next_segment_index >= len(self._route_node_ids) - 1):
            response.message = 'No unfinished route to resume. Call ~/start to begin a route.'
            return response
        if not (self.compute_and_track_route_client.server_is_ready()
                and self.compute_path_to_pose_client.server_is_ready()
                and self.follow_path_client.server_is_ready()):
            response.message = 'Nav2 route tracking, planner and controller action servers must be available.'
            return response

        # Resume the snapshot from the last start, even if node_ids was edited.
        node_ids = self._route_node_ids
        self._pending_node_ids = node_ids
        self._running = True
        response.success = True
        remaining = node_ids[self._next_segment_index:]
        if self._resume_node_id is not None:
            remaining = [self._resume_node_id] + remaining[1:]
        response.message = f'Route following accepted for remaining node IDs: {remaining}'
        return response

    def run_pending_route(self):
        if self._pending_node_ids is None:
            return
        node_ids = self._pending_node_ids
        self._pending_node_ids = None
        try:
            for index in range(self._next_segment_index, len(node_ids) - 1):
                start_node = (self._resume_node_id if self._resume_node_id is not None
                              else node_ids[index])
                end_node = node_ids[index + 1]
                self.info(f'Following route segment {start_node} -> {end_node}.')
                if not self._follow_segment(start_node, end_node):
                    self.warn(f'Route stopped on segment {start_node} -> {end_node}. '
                              'Call ~/resume to continue or ~/start to restart the full route.')
                    return
                self._next_segment_index = index + 1
                self._resume_node_id = None
            self.info('All nodes were successfully traversed. Ready for another start request.')
        except Exception as exc:
            self.error(f'Route following failed: {exc}')
        finally:
            try:
                if rclpy.ok():
                    self.cancelTask()
            except Exception as exc:
                self.error(f'Failed to cancel remaining navigation tasks: {exc}')
            finally:
                self._running = False

    def _follow_segment(self, start_node, end_node):
        route_task = self._request_route(start_node, end_node)
        if route_task is None:
            return False

        follow_task = RunningTask.NONE
        route_complete = False
        route_result = None
        controller_complete = False
        controller_result = None
        last_edge = None
        while rclpy.ok():
            # These navigator calls spin callbacks, including parameter updates
            # and duplicate start requests, outside of the service callback.
            if not route_complete:
                route_complete = self.isTaskComplete(task=route_task)
                if route_complete:
                    route_result = self.getResult()

            # Observe preemption before sending any further paths to the
            # controller, so a cancelled controller task stays stopped.
            if follow_task != RunningTask.NONE and not controller_complete:
                controller_complete = self.isTaskComplete(task=follow_task)
                if controller_complete:
                    controller_result = self.getResult()

            stopped = ((route_complete and route_result != TaskResult.SUCCEEDED)
                       or (controller_complete and controller_result != TaskResult.SUCCEEDED))

            feedback = self.getFeedback(task=route_task)
            while feedback is not None:
                edge = (feedback.last_node_id, feedback.next_node_id, feedback.current_edge_id)
                if edge != last_edge:
                    self.info(f'Passed node {edge[0]} towards {edge[1]} along edge {edge[2]}.')
                    last_edge = edge
                # The final target remains pending until BOTH actions succeed.
                # Intermediate nodes provide a checkpoint within a long segment.
                if not feedback.rerouted and feedback.last_node_id != end_node:
                    self._resume_node_id = feedback.last_node_id
                if not stopped and (follow_task == RunningTask.NONE or feedback.rerouted):
                    if not feedback.path.poses:
                        feedback = self.getFeedback(task=route_task)
                        continue
                    path = feedback.path
                    if follow_task == RunningTask.NONE:
                        path = self._connect_to_route(path)
                        if path is None:
                            return False
                        # Planning spins callbacks too; do not dispatch motion
                        # if tracking was cancelled while making the connection.
                        if self.isTaskComplete(task=route_task):
                            route_complete = True
                            route_result = self.getResult()
                            if route_result != TaskResult.SUCCEEDED:
                                return False
                    self.info(f'Sending {len(path.poses)} path poses to the controller.')
                    follow_task = self.followPath(path)
                    if follow_task is None:
                        return False
                    controller_complete = False
                    controller_result = None
                feedback = self.getFeedback(task=route_task)

            if stopped:
                error_code, error_msg = self.getTaskError()
                route_state = route_result if route_complete else 'in progress'
                controller_state = controller_result if controller_complete else 'in progress'
                self.warn(f'Navigation interrupted: route={route_state}, '
                          f'controller={controller_state}, '
                          f'error_code={error_code}, error_msg={error_msg}.')
                return False
            if route_complete and controller_complete:
                return True
            if route_complete and follow_task == RunningTask.NONE:
                self.error('Route tracking completed without providing a path to follow.')
                return False
        return False

    def _request_route(self, start_node, end_node):
        # BasicNavigator.getAndTrackRoute() has an unbounded goal-response
        # wait. Keep this handshake bounded so a missing response cannot
        # leave start/resume permanently busy without a diagnostic.
        self.clearPreviousState()
        self.route_goal_handle = None
        self.route_result_future = None
        self.route_feedback.clear()
        token = object()
        self._route_request_token = token

        goal = ComputeAndTrackRoute.Goal()
        goal.start_id = start_node
        goal.goal_id = end_node
        goal.use_poses = False
        goal.use_start = True

        def feedback_callback(message):
            if self._route_request_token is token:
                self._routeFeedbackCallback(message)

        self.info(f'Requesting route tracking {start_node} -> {end_node}; '
                  'waiting up to 10 seconds for goal acceptance.')
        future = self.compute_and_track_route_client.send_goal_async(goal, feedback_callback)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self._route_request_token = None

            def cancel_late_goal(completed):
                try:
                    handle = completed.result()
                    if handle is not None and handle.accepted:
                        handle.cancel_goal_async()
                except Exception as exc:
                    self.error(f'Could not cancel a late route goal response: {exc}')

            # Cancel this specific goal if its acknowledgement arrives later;
            # do not leave an unowned tracker running or mix its feedback into
            # a subsequent start/resume request.
            future.add_done_callback(cancel_late_goal)
            self.error('Route goal acknowledgement timed out after 10 seconds. '
                       'No controller path was sent. Check route_server and ROS action transport; '
                       'then call ~/start or ~/resume to retry.')
            return None

        self.route_goal_handle = future.result()
        if self.route_goal_handle is None or not self.route_goal_handle.accepted:
            self._route_request_token = None
            self.error('Route tracking goal was rejected by route_server.')
            return None
        self.route_result_future = self.route_goal_handle.get_result_async()
        self.info('Route tracking goal accepted. Waiting for a route path.')
        return RunningTask.COMPUTE_AND_TRACK_ROUTE

    def _connect_to_route(self, route_path):
        # A controller cannot join a distant route using its local costmap.
        # Plan the first mile from the current localized pose to the saved
        # route start. This also makes a full restart visit the original start
        # instead of letting path pruning skip earlier sections of the route.
        route_start = deepcopy(route_path.poses[0])
        route_start.header = deepcopy(route_path.header)
        self.info('Planning a connection from the robot to the route start.')
        connection = self.getPath(PoseStamped(), route_start, use_start=False)
        if connection is None or not connection.poses:
            error_code, error_msg = self.getTaskError()
            self.error(f'Cannot connect to route: error_code={error_code}, error_msg={error_msg}.')
            return None
        if connection.header.frame_id != route_path.header.frame_id:
            self.error('Planner connection and route path use different frames.')
            return None
        path = deepcopy(route_path)
        path.poses = list(connection.poses) + list(path.poses[1:])
        return path


def main():
    rclpy.init()
    navigator = RouteNavigator()
    try:
        while rclpy.ok():
            rclpy.spin_once(navigator, timeout_sec=0.1)
            navigator.run_pending_route()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        navigator.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
