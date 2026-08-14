# NOTICE

## Third-Party Attribution

### PCB_lightgraph

- **Repository**: https://github.com/tomatorigid/PCB_lightgraph
- **Author**: tomatorigid
- **License**: MIT License

PhotoPCB Studio was developed after studying and referencing the PCB_lightgraph project. The core concepts of PCB layer decomposition (Top Copper, Top Mask, Top Silk, Bottom Mask), the LED light diffusion preview, and the overall workflow structure were inspired by this project.

The following aspects have been substantially rewritten or extended in PhotoPCBStudio:

- Complete UI redesign (IBM Carbon-inspired dark theme, new layout)
- Image preprocessing pipeline (Gaussian denoise + color quantization as first-class workflow step)
- Rewritten edge detection cache (content-hash based, thread-safe)
- Iterative Douglas-Peucker with safety limits (no recursion stack overflow)
- Rewritten BFS connected-component cleanup (pre-built gray array, bounded stack)
- Rewritten Gaussian blur (RGB32 scanLine, no pixel() calls)
- Async processing with debounce and pending-request queue
- Project file format (.pcblg) with full parameter serialization
- MSYS2 UCRT64 build environment and windeployqt packaging

PhotoPCB Studio is intended to be an independent, more photo-focused tool rather than a copy of the original project.

---

PCB_lightgraph MIT License copy:

```
MIT License

Copyright (c) tomatorigid

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
