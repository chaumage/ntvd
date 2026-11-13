# optimize (planned)

Design-variable optimization over analyses: objectives and constraints on results, with
infeasible and `beyond_envelope` points penalized rather than turned into NaN. This directory
is reserved; it contains no code yet.

It will depend on `analysis` and the core solver. The first methods will be Nelder-Mead and
CMA-ES, with population methods such as NSGA-II later.
