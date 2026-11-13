# gui (planned)

Desktop graphical front end: the vehicle diagram, component editors, results and plots. This
directory is reserved; it contains no code yet.

It will depend on the same public layers as the CLI (`model`, `solve`, `analysis`,
`optimize`, `io`) and will own its own framework, event loop and allocation. None of that
belongs in `src/core/`.
