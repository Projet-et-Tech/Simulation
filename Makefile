.PHONY: all test install environment debug clean run runCameraReel runCameraSimu

# Default target
all: environment

# OS detection
OS := $(shell uname -s)

# Install dependencies
install:
	pip install -r requirements.txt

# Set up development environment
environment:
	@echo "> Creating virtual environment"
	@if ! command -v python3-venv &> /dev/null || ! command -v python3-tk &> /dev/null; then \
		echo "> Installing required system packages"; \
		if [[ "$(OS)" == "Linux" ]]; then \
			if [[ -f /etc/arch-release ]]; then \
				echo "> Installing for Arch Linux"; \
				sudo pacman -S python-virtualenv python-tk -y; \
			else \
				echo "> Assuming Ubuntu/Debian"; \
				sudo apt-get update; \
				sudo apt-get install python3-venv python3-tk -y; \
			fi \
		elif [[ "$(OS)" == "Darwin" ]]; then \
			echo "> Installing for macOS"; \
			brew install python; \
		elif [[ "$(OS)" == "CYGWIN"* || "$(OS)" == "MINGW"* || "$(OS)" == "MSYS"* ]]; then \
			echo "> Installing for Windows"; \
			echo "> Ensure Python and pip are installed"; \
		else \
			echo "> Unsupported OS: $(OS)"; \
			exit 1; \
		fi \
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
