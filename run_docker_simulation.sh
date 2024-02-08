docker-compose -f docker-compose-simulation.yml down --remove-orphans

if ! docker-compose -f docker-compose-simulation.yml up -d; then
    echo "Error starting docker. Do you have a VPN running? This may interfere with docker."
fi
