# arc-wiki
The wiki for the Autonomous Robotics Club's project teams.

## Running locally

This is also documented on the quickstart page.

```
docker run --rm -v "$PWD":/usr/src/app -w /usr/src/app ruby:2.6 bundle install
```

```
docker-compose -f ./docker/docker-compose.build-image.yml build
```

```
docker-compose -f ./docker/docker-compose.default.yml up
```

## Contributing

See the [quick-start](https://wiki.purduearc.com/wiki/contributing/quick-start) page
