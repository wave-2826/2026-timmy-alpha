FIELD_TYPE = "andymark"

PHOTON_IPS = [
    "10.28.26.203:5800",
    "10.28.26.204:5800"
]

import sys

if len(sys.argv) < 2:
    print("Usage: uploadTagMap.py <name>")
    sys.exit(1)

name = sys.argv[1]

tag_map_path = f"src/main/deploy/apriltags/{FIELD_TYPE}/{name}.json"

# Content-Disposition: form-data; name="data"; filename="tags.json"
# Content-Type: application/json
API_URL = "/api/settings/aprilTagFieldLayout"

with open(tag_map_path, "rb") as f:
    data = f.read()

import requests

for ip in PHOTON_IPS:
    url = f"http://{ip}{API_URL}"
    print(f"Uploading to {url}...")
    response = requests.post(url, files={"data": (f"{name}.json", data, "application/json")})
    if response.status_code == 200:
        print(f"Successfully uploaded to {ip}")
    else:
        print(f"Failed to upload to {ip}: {response.status_code} - {response.text}")