import json
import time
from pathlib import Path
import bosdyn.client
import bosdyn.client.util
from bosdyn.api import world_object_pb2
from bosdyn.client.world_object import WorldObjectClient

# Load registry.json from the same folder as this script
REGISTRY_PATH = Path(__file__).resolve().parent / "APRILTAG_Registry.json"

def load_registry():
    with open(REGISTRY_PATH, "r", encoding="utf-8") as f:
        return json.load(f)

def main():
    mailroom_registry = load_registry()

    sdk = bosdyn.client.create_standard_sdk("MailroomTagReader")
    robot = sdk.create_robot("192.168.80.3")
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)

    print("Scanning for AprilTags... (Press Ctrl+C to stop)")
    try:
        while True:
            response = world_object_client.list_world_objects(
                object_type=[world_object_pb2.WORLD_OBJECT_APRILTAG]
            )

            for obj in response.world_objects:
                tag_id = obj.apriltag_properties.tag_id
                # Convert integer tag_id to string to match JSON keys
                info = mailroom_registry.get(str(tag_id), {"error": f"Tag {tag_id} not in registry.json"})
                print(f"[Detected Tag ID: {tag_id}] -> {info}")

            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopped scanning.")

if __name__ == "__main__":
    main()