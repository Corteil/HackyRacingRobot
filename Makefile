PYTHON   := python3
REPO     := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
LOG_FILE := $(REPO)logs/robot.log
FIRMWARE := $(REPO)yukon_firmware_and_software/main.py

.PHONY: help upload test logs tail clean

help:
	@echo "Usage: make <target>"
	@echo ""
	@echo "  upload   Upload MicroPython firmware to the Yukon"
	@echo "  test     Run all no-hardware tests"
	@echo "  logs     Show last 50 lines of robot.log"
	@echo "  tail     Follow robot.log in real time"
	@echo "  clean    Remove __pycache__ trees"

upload:
	$(PYTHON) $(REPO)tools/upload.py $(FIRMWARE)

test:
	@echo "=== Yukon protocol ==="
	$(PYTHON) $(REPO)tools/test_main.py --dry-run
	@echo ""
	@echo "=== LD06 LiDAR ==="
	$(PYTHON) $(REPO)tools/test_ld06.py -u
	@echo ""
	@echo "=== GNSS package ==="
	$(PYTHON) $(REPO)tools/test_gnss.py
	@echo ""
	@echo "=== GPS dry-run ==="
	$(PYTHON) $(REPO)tools/test_gps.py --dry-run
	@echo ""
	@echo "=== ArUco detection ==="
	$(PYTHON) $(REPO)tools/test_aruco.py
	@echo ""
	@echo "=== LED strip ==="
	$(PYTHON) $(REPO)tools/test_leds.py --dry-run
	@echo ""
	@echo "=== Robot integration ==="
	$(PYTHON) $(REPO)tools/test_robot.py
	@echo ""
	@echo "All tests complete."

logs:
	@tail -50 $(LOG_FILE)

tail:
	@tail -f $(LOG_FILE)

clean:
	find $(REPO) -type d -name __pycache__ -exec rm -rf {} +
