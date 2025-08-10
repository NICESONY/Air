# ws_client.py
import asyncio
import websockets

async def main():
    uri = "ws://127.0.0.1:8000/ws/sensor"
    async with websockets.connect(uri) as ws:
        while True:
            # 서버가 값을 보낼 때까지 대기
            data = await ws.recv()
            print("Received:", data)

if __name__ == "__main__":
    asyncio.run(main())
