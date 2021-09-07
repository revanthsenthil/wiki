---
title: Contribution Quickstart
layout: article
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
7. Before you submit changes, view them by running them locally (directions in next section)
7. Submit a pull request to the ARC wiki GitHub
8. Editors may request changes. After that, it will be accepted and reflected on the website

### Using GitPod

#### Forking and Setting Up GitPod

1. Fork [this](https://github.com/purdue-arc/wiki) repository (top right button) to add it to your GitHub account.
2. Install the [GitPod Extension](https://chrome.google.com/webstore/detail/gitpod-dev-environments-i/dodmmooeoklaejobgleioelladacbeki?hl=en).
3. Go to the forked repository, connect GitPod to your GitHub account, and open the repository in GitPod.
4. Open the terminal:
      - Set up forked repository and call it upstream\*: `git remote add upstream https://github.com/whoever/whatever.git`
      - Install all packages necessary:  `bundle install`
      - Pull new changes in from GitHub: `git fetch upstream` &#8594; `git checkout master` &#8594; `git rebase upstream/master`

> \* These commands are not needed once the forked repository is set up the first time you use GitPod.

#### Writing the Page: 

1. Create your file under the folder of interest in the `wiki` folder.
2. Use [kebab-case](https://textcaseconvert.com/blog/kebab-case/) to appropriately name your file under a self-describing name (e.g `this-is-an-example-page.md`). 
3. Write your page in [markdown](https://www.markdownguide.org/cheat-sheet). The formatting can be viewed in the [wiki template]({% link wiki/contributing/template.md %}) provided.
4. Link your article in `_data/navigation.yml` under the `wiki` heading.
5. If you added your name as the author, add yourself to the \_data/authors.yml page to get some credit for your work.
6. Before committing your changes to GitHub, view them in the browser by running `jekyll serve` in the terminal.

#### Comitting and Opening a PR

1. Stage the files to commit: `git add .`
2. Commit all files to GitHub: `git commit -m "Write comments here"`
3. Push changes into local repository: `git push`
4. Submit a [pull request](https://docs.github.com/en/github/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request) to ARC wiki on GitHub and let a member of the Leadership Team know. Changes may be requested.


## Running Locally

If you want to check your changes on a local, development server you can either use Docker or Ruby.

### Docker

1. [Install Docker](https://docs.docker.com/install/).

2. Generate _Gemfile.lock_:

   ```bash
   docker run --rm -v "$PWD":/usr/src/app -w /usr/src/app ruby:2.6 bundle install
   ```

3. Build Docker image:

   ```bash
   docker-compose -f ./docker/docker-compose.build-image.yml build
   ```

### Ruby

Check out this [tutorial](https://docs.github.com/en/pages/setting-up-a-github-pages-site-with-jekyll/testing-your-github-pages-site-locally-with-jekyll).
