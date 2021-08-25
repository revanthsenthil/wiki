---
layout: article
title: Contribution Quickstart
key: contribute-quickstart
---

## New Page

To add a page, follow these steps:

1. Fork [this](https://github.com/purdue-arc/wiki) repository.
2. Add a copy of `wiki/contributing/template.md` to the project folder of interest in the `wiki` folder
3. Write your page in [markdown](https://www.markdownguide.org/cheat-sheet)
4. Rename the .md file to a [kebab-case](https://textcaseconvert.com/blog/kebab-case/) version of your title (e.g `this-is-an-example.md`)
5. Link your article in `_data/navigation.yml` under the `wiki` heading
6. If you added your name as the author, add yourself to the `_data/authors.yml` page to get some credit for your work
7. Before you submit changes, view your changes by testing the site locally:

- If Ruby is installed, you can run [jekyll normally](https://docs.github.com/en/pages/setting-up-a-github-pages-site-with-jekyll/testing-your-github-pages-site-locally-with-jekyll)
- Or you can use docker and run the following commands:

```
docker run --rm -v "$PWD":/usr/src/app -w /usr/src/app ruby:2.6 bundle install
```

```
docker-compose -f ./docker/docker-compose.build-image.yml build
```

8. Submit a pull request to the wiki GitHub
9. Editors may request changes. After that, it will be accepted and reflected on the website
