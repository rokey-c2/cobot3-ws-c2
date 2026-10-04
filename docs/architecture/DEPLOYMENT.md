# Architecture site deployment

The complete static site is stored in `docs/architecture/site`.
The files were copied from the existing Netlify production deployment to preserve the current layout and original Archify viewers.

## One-time Netlify setup

Link the existing Netlify project `cobot3-ws-c2-architectures` to this repository.
- Production branch: `main`
- Base directory: leave empty (repository root)
- Build command: leave empty (ready-to-publish HTML)
- Publish directory: `docs/architecture/site`

The root `netlify.toml` also declares the publish directory.
Git integration must be configured in Netlify; committing this configuration alone does not enable automatic deployment.

## Updating the site

1. Replace the completed site files in `docs/architecture/site`.
2. Keep `index.html` and all referenced viewer HTML files together. If filenames change, update the iframe paths and the JavaScript `paths` mapping in `index.html`.
3. Open a PR and merge into `main` according to repository rules.
4. Check the production deployment on the existing Netlify project and verify all four menus and both themes.

Uploading an archive elsewhere in the repository does not update the site. The deployed content comes from `docs/architecture/site`.
The SVG files in `docs/architecture/diagrams` are README images; update those separately when the diagrams change.
Robot source-code changes do not regenerate architecture diagrams automatically.
