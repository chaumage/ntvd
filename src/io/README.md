# io (planned)

File formats and serialization: strictly validated specs in, results out. This directory is
reserved; it contains no code yet.

It will depend on `model` (and `base` for units), never on the user interface. Keeping I/O
here is what lets the core stay free of file formats and allocation-heavy parsing.
