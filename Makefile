# Detect operating system
ifeq ($(OS),Windows_NT)
    DETECTED_OS := Windows
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        DETECTED_OS := Linux
        LINUX_DISTRO := $(shell cat /etc/os-release | grep -E "^ID=" | cut -d'=' -f2 | tr -d '"')
    endif
    ifeq ($(UNAME_S),Darwin)
        DETECTED_OS := macOS
    endif
endif

# Python and pip commands
ifeq ($(DETECTED_OS),Windows)
    PYTHON := python
    PIP := pip
    VENV_ACTIVATE := .venv\Scripts\activate
    VENV_CREATE := python -m venv .venv
else
    PYTHON := python3
    PIP := pip3
    VENV_ACTIVATE := .venv/bin/activate
    VENV_CREATE := $(PYTHON) -m venv .venv
endif

.PHONY: all test install environment debug clean run runCameraReel runCameraSimu

# Default target
all: environment

# Install dependencies
install:
	$(PIP) install -r requirements.txt

# Set up development environment
environment:
	@echo "> Creating virtual environment for $(DETECTED_OS)"
	
ifeq ($(DETECTED_OS),Linux)
    ifeq ($(LINUX_DISTRO),arch)
		@if ! command -v python-venv &> /dev/null || ! command -v tk &> /dev/null; then \
			echo "> Installing required system packages for Arch Linux"; \
			sudo pacman -S --noconfirm python-venv tk; \
		fi
    else
		@if ! command -v python3-venv &> /dev/null || ! command -v python3-tk &> /dev/null; then \
			echo "> Installing required system packages for Ubuntu/Debian"; \
			sudo apt-get update; \
			sudo apt-get install python3-venv python3-tk -y; \
		fi
    endif
endif

ifeq ($(DETECTED_OS),Windows)
	@if not exist .venv (
		$(PYTHON) -m venv .venv
	)
else
	@if [ ! -d ".venv" ]; then \
		$(VENV_CREATE); \
	fi
endif
	
	@echo "> Activating virtual environment and installing dependencies"
	
ifeq ($(DETECTED_OS),Windows)
	@.venv\Scripts\activate && $(PIP) install -r requirements.txt
else
	@. $(VENV_ACTIVATE) && $(PIP) install -r requirements.txt
endif
	
	@echo "> Environment setup complete for $(DETECTED_OS)"

# Clean up virtual environment
clean:
ifeq ($(DETECTED_OS),Windows)
	@echo Are you sure you want to remove the virtual environment? (Type 'y' to confirm)
	@read confirm && if [ "$confirm" = "y" ]; then \
		rmdir /s /q .venv; \
		echo "> Virtual environment removed"; \
	else \
		echo "> Aborted"; \
	fi
else
	@read -p "> Remove virtual environment? (y/N) " answer; \
	if [ "$answer" = "y" ] || [ "$answer" = "Y" ]; then \
		rm -rf .venv; \
		echo "> Virtual environment removed"; \
	else \
		echo "> Aborted"; \
	fi
endif

# Run specific scripts
runCameraReel:
ifeq ($(DETECTED_OS),Windows)
	@.venv\Scripts\activate && $(PYTHON) src\mainCameraReel.py
else
	@. $(VENV_ACTIVATE) && $(PYTHON) src/mainCameraReel.py
endif

runCameraSimu:
ifeq ($(DETECTED_OS),Windows)
	@.venv\Scripts\activate && $(PYTHON) src\mainCameraSimu.py
else
	@. $(VENV_ACTIVATE) && $(PYTHON) src/mainCameraSimu.py
endif

run:
ifeq ($(DETECTED_OS),Windows)
	@.venv\Scripts\activate && $(PYTHON) src\main.py
else
	@. $(VENV_ACTIVATE) && $(PYTHON) src/main.py
endif

# Placeholder for testing (add your test command)
test:
ifeq ($(DETECTED_OS),Windows)
	@.venv\Scripts\activate && $(PYTHON) src\test.py
else
	@. $(VENV_ACTIVATE) && $(PYTHON) src/test.py
endif

# Debug target (can be customized)
debug:
	@echo "> Debug mode for $(DETECTED_OS) (not fully implemented)"
