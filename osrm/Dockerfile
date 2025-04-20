FROM ghcr.io/project-osrm/osrm-backend:latest

COPY prepare_osrm_data.sh ./prepare_osrm_data.sh

RUN apk update && apk add --no-cache curl bash

RUN chmod +x ./prepare_osrm_data.sh && ./prepare_osrm_data.sh

CMD ["osrm-routed", "--algorithm", "ch", "/data/uruguay-latest.osrm"]