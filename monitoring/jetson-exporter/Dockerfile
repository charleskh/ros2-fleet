# Jetson tegra-metrics exporter (M6 Rung 2). Tiny image: just the jtop client + prometheus_client.
# Runs on hyperion; connects to the host's jtop.service via a mounted /run/jtop.sock.
FROM python:3.11-slim

# jetson-stats PINNED to the host's version — the jtop client and the jtop.service it talks to must
# match (protocol compatibility). Host runs 4.3.2 (`pip3 show jetson-stats` on hyperion).
RUN pip install --no-cache-dir jetson-stats==4.3.2 prometheus_client

COPY jetson_exporter.py /jetson_exporter.py
EXPOSE 9809
CMD ["python3", "/jetson_exporter.py"]
