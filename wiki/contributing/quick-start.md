---
layout: article
title: Contribution Quickstart
key: contribute-quickstart
---

## New Page

To add a page, follow these steps:

1. Fork [this](https://github.com/purdue-arc/wiki) repository.
2. Add a copy of `wiki/contributing/template.md` to the project folder of interest in the `wiki` folder
3. Write your page in [markdown](https://www.markdownguide.org/cheat-sheet). Some recommended methods:

- Connect [Prose.io](http://prose.io/) to your Github account and write from your browser
- Clone the repository to your device. We recommend using the Atom editor with the Markdown-Writer and Markdown-Image-Helper packages.

4. Rename the .md file to a [kebab-case](https://textcaseconvert.com/blog/kebab-case/) version of your title (e.g `this-is-an-example.md`)
5. Link your article in `_data/navigation.yml` under the `wiki` heading
6. If you added your name as the author, add yourself to the \_data/authors.yml page to get some credit for your work
7. Submit a pull request to the ARC wiki GitHub
8. Editors may request changes. After that, it will be accepted and reflected on the website

## Running Locally

If you want to check your changes on a local, development server then follow these steps:

1. [Install Docker](https://docs.docker.com/install/).

2. Generate _Gemfile.lock_:

   ```bash
   docker run --rm -v "$PWD":/usr/src/app -w /usr/src/app ruby:2.6 bundle install
   ```

3. Build Docker image:

   ```bash
   docker-compose -f ./docker/docker-compose.build-image.yml build
   ```
