.PHONY: all test install environment debug clean run runCameraReel runCameraSimu

# Default target
all: environment

# Install dependencies
install:
	pip install -r requirements.txt

# Set up development environment
environment:
	@echo "> Creating virtual environment"
	@if [[ "$(uname)" == "Linux" ]]; then \
		if command -v pacman &> /dev/null; then \
			echo "> Installing required system packages for Arch"; \
			sudo pacman -Syu python-virtualenv python-tk --noconfirm; \
		elif command -v apt-get &> /dev/null; then \
			echo "> Installing required system packages for Debian/Ubuntu"; \
			sudo apt-get update; \
			sudo apt-get install python3-venv python3-tk -y; \
		else \
			echo "> Unsupported Linux distribution"; \
			exit 1; \
		fi \
	elif [[ "$(uname)" == "Darwin" ]]; then \
		echo "> macOS detected, ensure Python 3 and venv are installed"; \
	elif [[ "$(uname -o)" == "Msys" ]]; then \
		echo "> Windows detected"; \
		echo "> You might want to install required Python packages manually"; \
		@echo "> Make sure Python 3 is added to your PATH."; \
	else \
		echo "> Unsupported OS"; \
		exit 1; \
	fi
	@python3 -m venv .venv
	@echo "> Activating virtual environment and installing dependencies"
	@. .venv/bin/activate && pip install -r requirements.txt
	@echo "> Environment setup complete"

# Clean up virtual environment
clean:
	@read -p "> Remove virtual environment? (y/N) " answer; \
	if [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then \
		rm -rf .venv; \
		echo "> Virtual environment removed"; \
	else \
		echo "> Aborted"; \
	fi

# Run specific scripts
runCameraReel:
	@. .venv/bin/activate && python3 src/mainCameraReel.py

runCameraSimu:
	@. .venv/bin/activate && python3 src/mainCameraSimu.py

run:
	@. .venv/bin/activate && python3 src/main.py

# Placeholder for testing (add your test command)
test:
	@. .venv/bin/activate && python3 src/test.py

# Debug target (can be customized)
debug:
	@echo "> Debug mode (not implemented)"
