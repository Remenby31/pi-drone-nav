#!/bin/bash
#
# EMERGENCY STOP - Disarm drone immediately
#
# Usage: ./emergency_stop.sh [raspberry_ip]
#
# Default IP: 192.168.1.114
#

# Colors for terminal
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuration
RASPBERRY_IP="${1:-192.168.1.114}"
API_PORT="8080"
API_URL="http://${RASPBERRY_IP}:${API_PORT}/api/emergency/disarm"

echo -e "${RED}"
echo "========================================"
echo "    !!! EMERGENCY DISARM !!!"
echo "========================================"
echo -e "${NC}"
echo "Target: ${API_URL}"
echo ""

# Send emergency disarm command
response=$(curl -s -w "\n%{http_code}" -X POST "${API_URL}" \
    -H "Content-Type: application/json" \
    --connect-timeout 2 \
    --max-time 5)

# Extract HTTP code (last line)
http_code=$(echo "$response" | tail -n1)
body=$(echo "$response" | sed '$d')

if [ "$http_code" = "200" ]; then
    echo -e "${GREEN}========================================"
    echo "         DISARMED SUCCESSFULLY"
    echo "========================================${NC}"
    echo ""
    echo "Response: $body"
else
    echo -e "${RED}========================================"
    echo "         DISARM FAILED!"
    echo "========================================${NC}"
    echo ""
    echo "HTTP Code: $http_code"
    echo "Response: $body"
    echo ""
    echo -e "${YELLOW}Retrying...${NC}"

    # Retry 3 times
    for i in 1 2 3; do
        sleep 0.2
        response=$(curl -s -w "\n%{http_code}" -X POST "${API_URL}" \
            -H "Content-Type: application/json" \
            --connect-timeout 1 \
            --max-time 2)
        http_code=$(echo "$response" | tail -n1)

        if [ "$http_code" = "200" ]; then
            echo -e "${GREEN}Retry $i: DISARMED${NC}"
            exit 0
        else
            echo -e "${RED}Retry $i: Failed (HTTP $http_code)${NC}"
        fi
    done

    echo ""
    echo -e "${RED}!!! ALL RETRIES FAILED !!!${NC}"
    echo "Check connection to Raspberry Pi at ${RASPBERRY_IP}"
    exit 1
fi
