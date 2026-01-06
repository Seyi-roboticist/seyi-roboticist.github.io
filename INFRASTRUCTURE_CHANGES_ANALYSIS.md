# Infrastructure Changes Analysis: Portfolio Branch vs Main

**Analysis Date:** January 5, 2026  
**Current Branch:** Portfolio  
**Comparison Target:** main  
**Focus:** Core Template Infrastructure Only (No Project-Specific Assets)

---

## Executive Summary

The Portfolio branch introduces **three major infrastructure enhancements**:

1. **Math Support** - MathJax integration for LaTeX rendering
2. **Interactive Data Visualization** - Plotly.js for CSV-based charts
3. **Gallery & Media Management** - Improved video/image rendering with captions

These are pure infrastructure changes that enhance the template's capabilities without modifying any project-specific content.

---

## 1. Math Support (MathJax)

### Location
[_layouts/default.html](_layouts/default.html#L20-L36)

### Changes
Added MathJax CDN script and configuration:

```html
<!-- MathJax -->
<script>
    window.MathJax = {
        tex: {
            inlineMath: [['$', '$'], ['\\(', '\\)']],
            displayMath: [['$$', '$$'], ['\\[', '\\]']],
            processEscapes: true
        },
        svg: {
            fontCache: 'global'
        }
    };
</script>
<script id="MathJax-script" async 
    src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js">
</script>
```

### Impact
- **Inline math:** `$E=mc^2$` renders as: $E=mc^2$
- **Display math:** `$$\frac{d}{dx}$$` renders centered
- Supports full LaTeX syntax
- Automatic rendering on page load
- No project changes required; works globally

### Usage in Projects
Projects can now include mathematical equations directly in markdown:
- Control theory equations
- Physics formulas
- Engineering calculations

---

## 2. Interactive Data Visualization (Plotly.js)

### Location
[_layouts/default.html](_layouts/default.html#L83-L85) - Library inclusion  
[_includes/interactive-plot.html](_includes/interactive-plot.html) - Main implementation  
[_includes/universal-plot-handler.html](_includes/universal-plot-handler.html) - Advanced plots  
[_includes/servos-plots.html](_includes/servos-plots.html) - Specialized example  

### Changes

#### 2.1 Plotly.js Library
Added CDN link in default layout:
```html
<!-- Plotly.js for interactive data visualization -->
<script src="https://cdn.plot.ly/plotly-2.26.0.min.js"></script>
```

#### 2.2 Core Plot Components

**interactive-plot.html** (~186 lines)
- Robust CSV data loading
- Automatic decimal data parsing
- Error handling with user-friendly messages
- Loading indicator with status updates
- Large dataset decimation (8000+ point handling)
- Responsive plot configuration

Key features:
- Polls for Plotly availability (10s timeout)
- Fetch data from configurable CSV files
- Parse numeric values with fallback handling
- Automatic length validation and truncation
- Fade-out loading overlay on completion

**universal-plot-handler.html** (~206 lines)
- Multi-plot per page support
- CSV parsing with header detection
- Column-by-column data extraction
- Color cycling for multiple traces
- BaseURL resolution for relative paths
- Detailed console logging for debugging

**servos-plots.html** (~162 lines)
- Specialized for 2-plot grids
- Cartesian path visualization
- Joint dynamics over time
- Specific CSV parsing for servo data

#### 2.3 Configuration Method

Projects enable plots by adding to their frontmatter:

```yaml
# Option 1: Simple interactive plot
interactive_plot: true
plot_config:
  title: "Performance Data"
  x_file: /assets/data/plots/myproject/x.csv
  y_file: /assets/data/plots/myproject/y.csv
  x_label: "Time (s)"
  y_label: "Value"

# Option 2: Universal multi-plot handler
plots:
  - title: "Plot 1"
    x_file: /assets/data/plots/file1.csv
    x_column: 0
    y_files:
      - file: /assets/data/plots/file2.csv
        column: 1
        label: "Dataset A"
  - title: "Plot 2"
    # ... more plots
plots_title: "Data Analysis"

# Option 3: Custom include
custom_plot: custom-plot-file.html
```

### CSV Data Files

Configuration in [_config.yml](_config.yml#L20-L22):
```yaml
# Include CSV files in build
include:
  - assets/data/plots
```

Sample data structure added:
- `assets/data/plots/ebarisbot/torque.csv`
- `assets/data/plots/parrot-minidrone/simulink_export_results.csv`
- `assets/data/plots/servos/robot_cartesian_simple.csv`
- `assets/data/plots/servos/robot_joint_simple.csv`

### Impact
- **Responsive design:** Auto-resizes to container
- **Theme integration:** Uses CSS variables for colors
- **Error resilience:** Graceful failure with detailed messages
- **Performance:** Decimates large datasets automatically
- **Accessibility:** Non-blocking loading with status feedback

---

## 3. Gallery & Media Management

### Location
[_layouts/project.html](_layouts/project.html#L165-L200) - Gallery rendering  
[_sass/_project.scss](_sass/_project.scss#L474-L508) - Styling

### Changes

#### 3.1 Media Type Detection

Old approach (required explicit type):
```yaml
gallery:
  - type: video
    file: demo.mp4
  - type: image
    file: photo.jpg
```

New approach (auto-detection by extension):
```yaml
gallery:
  - file: demo.mp4         # Auto-detects as video
  - file: demo.avi         # Also supports AVI
  - file: photo.jpg        # Auto-detects as image
    description: "Robot in action"
```

Implementation: [_layouts/project.html](_layouts/project.html#L168-L176)
```html
{% assign file_extension = media.file | split: '.' | last | downcase %}
{% if file_extension == 'mp4' or file_extension == 'webm' or 
      file_extension == 'avi' %}
    <!-- Render as video -->
{% else %}
    <!-- Render as image -->
{% endif %}
```

#### 3.2 Media Captions

New optional caption support:
```yaml
gallery:
  - file: image.jpg
    description: "Detailed caption for the image"
```

Renders as:
```html
<p class="media-caption">Detailed caption for the image</p>
```

#### 3.3 CSS Changes

**Container Styling** ([_project.scss](_sass/_project.scss#L474-L482)):
- Removed `overflow: hidden`
- Added `display: flex` with `flex-direction: column`
- Padding for better spacing
- Maintains centered alignment

**Media Elements** ([_project.scss](_sass/_project.scss#L484-L507)):

From (fixed height, cropped):
```scss
.gallery-image {
  height: 250px;
  object-fit: cover;  // Crops the image
}
```

To (responsive, contained):
```scss
.gallery-image {
  height: auto;
  max-width: 100%;
  object-fit: contain;  // Preserves aspect ratio
  display: block;
}
```

Same changes applied to:
- `.gallery-gif`
- `.gallery-video`

**Caption Styling** (new):
```scss
.media-caption {
  padding: var(--spacing-sm) var(--spacing-md);
  color: var(--text-secondary);
  font-size: var(--font-size-sm);
  text-align: center;
  background-color: var(--background-color);
  width: 100%;
}
```

#### 3.4 Responsive Updates

[_sass/_responsive.scss](_sass/_responsive.scss#L264-L269):

From (fixed height):
```scss
@media (max-width: 768px) {
  .gallery-image {
    height: 200px;
  }
}
```

To (responsive with max-height):
```scss
@media (max-width: 768px) {
  .gallery-image {
    height: auto;
    max-height: 400px;
  }
}
```

### Impact
- **Better aspect ratio preservation:** No more cropped images
- **Video support expanded:** AVI format now works
- **Optional captions:** Better documentation of media
- **Type detection:** Simpler YAML frontmatter
- **Mobile friendly:** Responsive heights on smaller screens

---

## 4. Project Listing Improvements

### Location
[index.md](index.md) - Featured projects  
[projects.md](projects.md) - All projects list

### Changes

#### 4.1 Sorting by Date (Reverse)

Before:
```liquid
{% for project in site.projects limit:9 %}
```

After:
```liquid
{% assign sorted_projects = site.projects | sort: "date" | reverse %}
{% for project in sorted_projects limit: 9 %}
```

Applied to:
1. Featured projects grid (index.md)
2. Full projects list (projects.md)

### Impact
- **Latest first:** Most recent projects appear first
- **Consistent ordering:** Both pages use same logic
- **Date-dependent:** Requires `date:` frontmatter in projects

---

## 5. Configuration & Build Settings

### Location
[_config.yml](_config.yml)

### Changes

Added CSV inclusion directive:
```yaml
# Include CSV files in build
include:
  - assets/data/plots
```

### Impact
- Jekyll now copies CSV files during build
- Ensures data files are available to JavaScript
- Required for plot functionality

---

## 6. Additional Infrastructure Files

### New Files

| File | Purpose | Lines |
|------|---------|-------|
| [debug.md](debug.md) | Project listing debug page | 14 |
| [_includes/debug-plot-config.html](_includes/debug-plot-config.html) | Plot configuration debugger | 14 |
| [START_HERE.txt](START_HERE.txt) | User guide (not code) | 291 |

### START_HERE.txt
Comprehensive setup and usage guide including:
- Quick reference
- Version information
- Feature list
- File inventory

---

## 7. Migration Checklist

To apply these infrastructure changes to a branch without project migrations:

### ✅ Essential Files to Migrate

**Layout Files:**
- [ ] [_layouts/default.html](_layouts/default.html) - MathJax + Plotly
- [ ] [_layouts/project.html](_layouts/project.html) - Gallery improvements

**Include Files:**
- [ ] [_includes/interactive-plot.html](_includes/interactive-plot.html) - Interactive plots
- [ ] [_includes/universal-plot-handler.html](_includes/universal-plot-handler.html) - Multi-plot support
- [ ] [_includes/servos-plots.html](_includes/servos-plots.html) - Reference example
- [ ] [_includes/debug-plot-config.html](_includes/debug-plot-config.html) - Debugging tool

**Styling:**
- [ ] [_sass/_project.scss](_sass/_project.scss) - Gallery CSS updates
- [ ] [_sass/_responsive.scss](_sass/_responsive.scss) - Responsive media fixes

**Configuration:**
- [ ] [_config.yml](_config.yml) - Add CSV include directive

**Documentation:**
- [ ] [debug.md](debug.md) - Debug page
- [ ] [START_HERE.txt](START_HERE.txt) - User guide

### ✅ Recommended Project Updates

**For existing projects to use new features:**
1. Add `date:` field to frontmatter (for sorting)
2. Add optional `interactive_plot: true` if using plots
3. Simplify gallery YAML (remove `type:` field)
4. Add `description:` fields to gallery items

### ⚠️ Do NOT Migrate

- Any files in `_projects/` folder
- Any files in `assets/` (except `/assets/data/plots/` structure)
- Project-specific images, models, or schematics
- `projects.md` and `index.md` lists (content, not structure)

---

## 8. Backward Compatibility

All changes are **backward compatible**:

- ✅ Old gallery format (with `type:`) still works
- ✅ Gallery items without captions work fine
- ✅ Projects without `date:` field still display
- ✅ Projects without `interactive_plot:` field display normally
- ✅ Old plot includes still function as expected
- ✅ Inline/display math optional - doesn't break if not used

---

## 9. Summary Table

| Feature | Type | Files Changed | Impact |
|---------|------|---------------|--------|
| Math Support | Library | default.html | Enables LaTeX in markdown |
| Interactive Plots | JavaScript | interactive-plot.html, universal-plot-handler.html, servos-plots.html | CSV visualization |
| Plot Library | CDN | default.html | Plotly.js availability |
| Gallery Auto-Detection | Template Logic | project.html | Simplifies video/image markup |
| Media Captions | CSS + HTML | project.html, _project.scss | Optional image/video descriptions |
| Responsive Media | CSS | _project.scss, _responsive.scss | Better aspect ratio on mobile |
| Project Sorting | Liquid | index.md, projects.md | Chronological listing |
| CSV Build Support | Config | _config.yml | Data file inclusion |

---

## 10. Testing Recommendations

After migration, verify:

1. **Math:** Add `$E=mc^2$` to a project, confirm rendering
2. **Plots:** Add sample CSV file and plot config, check visualization
3. **Gallery:** Test with .mp4, .avi, and image files
4. **Captions:** Add description field, verify display
5. **Mobile:** Check responsive layouts at 768px breakpoint
6. **Performance:** Monitor plot load time with large CSV files
7. **Backwards compatibility:** Ensure old projects still display correctly

---

## Final Notes

All changes maintain the template's core design philosophy:
- **Modular:** Features can be used independently
- **Configurable:** Projects opt-in to new features
- **Non-intrusive:** Existing content unaffected
- **Well-documented:** Clear configuration examples

No data from projects or assets have been modified—only infrastructure enhancements.

