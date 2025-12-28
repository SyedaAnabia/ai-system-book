from qdrant_client import QdrantClient

client = QdrantClient(
    url="https://e2496799-ca00-4963-a5b3-aea8a5d6a2e9.us-east4-0.gcp.cloud.qdrant.io",
    api_key="eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.TtoO_1umrnf2YekRRdH5Sxzs4vyy7chmS3Epabrm6-E"
)

print("✅ Connected!")
print(client.get_collections())