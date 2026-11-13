# cli (planned)

Command-line front end for batch runs: compile a spec, run an analysis or optimization, and
write results. This directory is reserved; it contains no code yet.

It will consume the solver through `model`, `solve`, `analysis`, `optimize` and `io`, keeping
argument parsing and terminal output out of `src/core/`.
