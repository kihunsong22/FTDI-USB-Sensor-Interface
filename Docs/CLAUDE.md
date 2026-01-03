# Documentation Guidelines for Claude

## CRITICAL: Anti-Overdocumentation Policy

**KEEP IT MINIMAL - 1-2 documents MAX per package**
- For the 01-mpu6050-interface package: NO MORE THAN 2 DOCUMENTS TOTAL
- Don't create separate docs for every component/feature
- Don't overdocument with exhaustive details
- Engineers can read code - docs should be quick reference only

**Acceptable documentation structure:**
- Root `INSTALLATION.md` - Prerequisites for entire repository
- Package `README.md` - Quick overview + how to run
- That's it. Nothing more.

## Style Preferences

- Target audience: Engineers with technical background
- No beginner tutorials (e.g., "how to install Rust")
- Focus only on project-specific details
- Use tables and bullet points over lengthy paragraphs
- Quick reference format, not exhaustive tutorials

## Project-Specific Notes

### HDF5
- **Use 1.14.x only** - Rust `hdf5` crate does NOT support HDF5 2.0
- Path: `C:\Program Files\HDF_Group\HDF5\1.14.5`

### Architecture
- Three-program system: collector, visualizer, analyzer
- File-based data flow via HDF5 (SWMR mode)
- Separation of concerns: acquisition, visualization, analysis
