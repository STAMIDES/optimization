FROM ghcr.io/project-osrm/osrm-backend

COPY prepare_osrm_data.sh ./prepare_osrm_data.sh

RUN apt-get -y update && apt-get install -y curl

RUN chmod +x ./prepare_osrm_data.sh && ./prepare_osrm_data.sh

CMD ["osrm-routed", "--algorithm", "ch", "/data/uruguay-latest.osrm"]