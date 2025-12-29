# Contributing to Pi Drone Navigation

Thank you for your interest in contributing!

## How to Contribute

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Run tests (`python -m pytest tests/`)
5. Commit your changes (`git commit -m 'Add amazing feature'`)
6. Push to the branch (`git push origin feature/amazing-feature`)
7. Open a Pull Request

## Development Setup

```bash
git clone https://github.com/yourusername/pi-drone-nav.git
cd pi-drone-nav
python -m venv venv
source venv/bin/activate
pip install -e ".[dev]"
```

## Code Style

- Follow PEP 8
- Use type hints
- Add docstrings to public functions
- Keep functions focused and small

## Testing

- Write tests for new features
- Ensure all tests pass before submitting PR
- Aim for good test coverage

## Safety

When contributing code that affects flight behavior:
- Document all changes thoroughly
- Test in simulation first
- Consider failsafe implications
- Never disable safety checks

## Questions?

Open an issue for discussion.
