FROM ghcr.io/project-osrm/osrm-backend:latest

COPY prepare_osrm_data.sh car.lua /opt/

RUN apk update && apk add --no-cache curl bash

RUN chmod +x ./prepare_osrm_data.sh && ./prepare_osrm_data.sh

CMD ["osrm-routed", "--algorithm", "ch", "--max-table-size", "5000", "/data/uruguay-latest.osrm"]
