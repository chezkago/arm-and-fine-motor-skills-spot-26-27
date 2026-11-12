import sys
import math
import time
import argparse

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry

from bosdyn.client.estop import EstopClient
from bosdyn.client.robot_command import RobotCommandClient, blocking_stand
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api import estop_pb2

# copied from estop SDK example
def verify_estop(robot):
    """Verify the robot is not estopped"""

    client = robot.ensure_client(EstopClient.default_service_name)
    if client.get_status().stop_level != estop_pb2.ESTOP_LEVEL_NONE:
        error_message = 'Robot is estopped. Please use an external E-Stop client, such as the' \
                        ' estop SDK example, to configure E-Stop.'
        robot.logger.error(error_message)
        raise Exception(error_message)

def arm_grasp(config):
    bosdyn.client.util.setup_logging(config.verbose)
    logger = bosdyn.client.util.get_logger()

    sdk = bosdyn.client.create_standard_sdk('ArmGraspClient')
    robot = sdk.create_robot(config.hostname)
    robot.authenticate(config.username, config.password)

    robot.time_sync.wait_for_sync()

    assert robot.has_arm()

    verify_estop(robot)

    assert not robot.is_estopped(), "Robot is estopped."

    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        robot.logger.info('Powering on robot... This may take a several seconds.')
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), 'Robot power on failed.'
        robot.logger.info('Robot powered on.')

        robot.logger.info('Commanding robot to stand...')
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec=10)
        robot.logger.info('Robot standing.')

        # Robot should be ready to use here.



        robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not robot.is_powered_on(), 'Robot power off failed.'
        robot.logger.info('Robot safely powered off.')
    

def main():
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args() 
    try:
        arm_grasp(options)
        return True
    except Exception as e:
        logger = bosdyn.client.util.get_logger()
        logger.error(f'Exception: {e}')
        return False


if __name__ == "__main__":
    if not main():
        sys.exit(1)