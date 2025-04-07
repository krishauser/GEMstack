import asyncio
import json
import logging
import websockets
import uuid
from datetime import datetime
from message_constants import ClientRole, MessageType, MissionEnum

# configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[logging.StreamHandler()]
)

# store connected clients with their roles
connected_clients = {}  # {websocket: {"id": client_id, "role": role}}
# store active summoning missions
active_missions = {}
# track executed summons to prevent duplicates
executed_summons = set()  # set of coordinate tuples (x, y)

async def handle_client(websocket):
    """handle a client connection."""
    client_id = str(uuid.uuid4())
    client_role = ClientRole.UNKNOWN

    # wait for initial registration message to determine role
    try:
        # set a timeout for registration
        registration_message = await asyncio.wait_for(websocket.recv(), timeout=10.0)
        data = json.loads(registration_message)

        if "role" in data:
            role_str = data["role"].lower()
            if role_str == "webapp":
                client_role = ClientRole.WEBAPP
            elif role_str == "server":
                client_role = ClientRole.SERVER
            elif role_str == "gemstack":
                client_role = ClientRole.GEMSTACK

        logging.info(f"new client connected: {client_id} with role {client_role}")

        # send acknowledgment
        await websocket.send(json.dumps({
            "type": MessageType.REGISTRATION_RESPONSE,
            "client_id": client_id,
            "role": client_role,
            "status": "connected"
        }))

        # store client with role
        connected_clients[websocket] = {"id": client_id, "role": client_role}

        # process messages
        async for message in websocket:
            logging.info(f"received message from {client_id} ({client_role}): {message}")

            try:
                data = json.loads(message)

                # add source role to the message for processing
                data["source_role"] = client_role
                data["source_id"] = client_id

                # handle different message types
                if "type" in data:
                    msg_type = data["type"]
                    if msg_type == MessageType.SUMMON:
                        await handle_summon_request(websocket, data, client_id, client_role)
                    else:
                        logging.warning(f"unknown or unimplemented message type: {msg_type}")
                else:
                    logging.warning("message missing 'type' field")

            except json.JSONDecodeError:
                logging.error(f"invalid JSON: {message}")
                await websocket.send(json.dumps({
                    "type": MessageType.ERROR,
                    "message": "invalid JSON format"
                }))

    except asyncio.TimeoutError:
        logging.warning(f"client {client_id} registration timed out")
        await websocket.send(json.dumps({
            "type": MessageType.ERROR,
            "message": "registration timed out. please identify your role."
        }))
        return
    except websockets.exceptions.ConnectionClosed:
        logging.info(f"client disconnected during registration: {client_id}")
    except Exception as e:
        logging.error(f"error during client handling: {str(e)}")
    finally:
        if websocket in connected_clients:
            del connected_clients[websocket]
            logging.info(f"client removed: {client_id} ({client_role})")

async def handle_summon_request(websocket, data, client_id, client_role):
    """process a summoning request with coordinates."""
    # note: this is a temporary check to ensure the request is coming from a trusted source
    # TODO: remove this check when we have a proper authentication mechanism
    if client_role != ClientRole.WEBAPP and client_role != ClientRole.SERVER:
        logging.error(f"unauthorized role ({client_role}) for summon request")
        await websocket.send(json.dumps({
            "type": MessageType.ERROR,
            "message": f"unauthorized role ({client_role}) for summon request"
        }))
        return

    if "coordinates" not in data:
        await websocket.send(json.dumps({
            "type": MessageType.ERROR,
            "message": "missing coordinates in summon request"
        }))
        return

    coords = data["coordinates"]
    # validate coordinates format
    if not all(k in coords for k in ["x", "y"]):
        logging.error(f"invalid coordinates: {coords}")
        await websocket.send(json.dumps({
            "type": MessageType.ERROR,
            "message": "coordinates must include x and y values"
        }))
        return

    # check if these coordinates have already been executed
    coord_tuple = (coords["x"], coords["y"])
    if coord_tuple in executed_summons:
        logging.error(f"summon to coordinates {coord_tuple} was already executed")
        await websocket.send(json.dumps({
            "type": MessageType.ERROR,
            "message": f"summon to coordinates {coord_tuple} was already executed",
            "source_role": ClientRole.SERVER
        }))
        return

    # create a new mission
    mission_id = str(uuid.uuid4())
    timestamp = datetime.now().isoformat()

    mission = {
        "id": mission_id,
        "client_id": client_id,
        "client_role": client_role,
        "coordinates": coords,
        "mission_enum": MissionEnum.DRIVE.value,
        "timestamp": timestamp
    }

    active_missions[mission_id] = mission
    executed_summons.add(coord_tuple)

    # send response to client
    await websocket.send(json.dumps({
        "type": MessageType.SUMMON_RESPONSE,
        "mission_id": mission_id,
        "mission_enum": MissionEnum.DRIVE.value,
        "timestamp": timestamp,
        "source_role": ClientRole.SERVER
    }))

    # Note: Not implementing broadcast to GEMstack clients yet
    logging.info(f"summon request processed for mission {mission_id} to coordinates ({coords['x']}, {coords['y']})")

async def main():
    """start the websocket server."""
    host = "localhost"
    port = 8765

    # updated serve call for newer websockets versions
    async with websockets.serve(handle_client, host, port):
        logging.info(f"websocket server started at ws://{host}:{port}")
        # keep the server running indefinitely
        await asyncio.Future()  # run forever

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.info("server shutdown initiated by user")
