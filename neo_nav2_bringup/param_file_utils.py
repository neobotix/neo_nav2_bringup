"""
This module provides a helper function to generate a final YAML parameter file from a template file
with substitutions applied. This helper is intended for YAML parameter files only. It uses the ROS 2 
ParameterFile class to perform substitutions on the template file. The resulting YAML is written to a 
final file in the system's temporary directory with the given file_name.

If cleanup_enabled is True, a shutdown handler is created to delete the final file when the launch 
process shuts down; otherwise, the file has to be manually deleted.

Author: Adarsh Karan K P
"""

import os
import tempfile
import yaml

from typing import Tuple
from launch import LaunchContext
from launch.actions import LogInfo, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnShutdown
from launch_ros.descriptions import ParameterFile

def generate_final_yaml(
    context: LaunchContext,
    template_path: str,
    file_name: str,
    cleanup_enabled: bool = True
) -> Tuple[str, object]:

    """
    Generate a final YAML file from a template parameter file with substitutions applied.

    Args:
        context: The LaunchContext from the launch system.
        template_path: The path to the YAML template file.
        file_name: The desired name for the final file (stored in the system temp directory).
        cleanup_enabled: If True, a shutdown handler is registered to delete the final file.

    Returns:
        A tuple (final_file_path, shutdown_action). If cleanup_enabled is False, shutdown_action is an empty list.

    Raises:
        Exception: If evaluation or file operations fail.
    """
    # Create a ParameterFile instance and evaluate substitutions
    param_file = ParameterFile(template_path, allow_substs=True)
    try:
        evaluated_path = param_file.evaluate(context)
    except Exception as e:
        raise Exception(f"Failed to evaluate parameter file: {template_path}. Error: {e}")
    
    # Define the final file path in the system temporary directory
    final_file_path = os.path.join(tempfile.gettempdir(), file_name)
    
    try:
        with open(evaluated_path, 'r') as src_file, open(final_file_path, 'w') as dest_file:
            dest_file.write(src_file.read())
    except Exception as e:
        raise Exception(f"Failed to create final YAML file: {final_file_path}. Error: {e}")
    
    # Log to acknowledge that the final file has been created
    log_action = LogInfo(msg=f"Final YAML file created at: {final_file_path}")
    log_action.execute(context)
    
    shutdown_action = []
    if cleanup_enabled:
        # Define the cleanup function that deletes the final file
        def cleanup_fn(context, *args, **kwargs):
            try:
                os.remove(final_file_path)
                return [LogInfo(msg=f"Deleted final YAML file: {final_file_path}")]
            except FileNotFoundError:
                return [LogInfo(msg=f"Final YAML file not found (already deleted): {final_file_path}")]
            except Exception as e:
                return [LogInfo(msg=f"Error deleting final YAML file: {final_file_path}. Error: {e}")]
        # Wrap the cleanup function in an OpaqueFunction and register it as a shutdown event
        shutdown_action = [RegisterEventHandler(
            OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup_fn)])
        )]
        
    return final_file_path, shutdown_action
