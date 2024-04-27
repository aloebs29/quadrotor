# Cross-compilation Docker image

To build (from this directory):
```
docker build --build-arg IMAGE_VERSION=1.75 -t ghcr.io/aloebs29/rust-thumbv7em-none-eabihf:1.75 .
```

To push (use github username + PAT as password):
```
docker login ghcr.io
docker push ghcr.io/aloebs29/rust-thumbv7em-none-eabihf:1.75
```
