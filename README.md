# Ardupilot Dataflash Log => Foxglove MCAP converter.

## What
This is a pure rust command-line utility to convert the Ardupilot binary log ("Dataflash log") to Foxglove's MCAP format.

In addition to /ardupilot topics, it attempts to emit up to three foxglove-specific topics:

- /foxglove/map_origin
- /foxglove/gps
- /foxglove/base_link_transform

With these topics, Foxglove's Map panel and 3D panel can work out of the box. 

## Usage

```bash
cargo install arducap
arducap <bin_file1>...
```

This will create .mcap files alongside the original .bin files, named similarly. 

**WARNING**: if an .mcap with that name exists, it will be overwritten!


## Why

What started as a hands-on study of formats, ended up a utility for our friend to review his ardupilot logs in the awesome Foxglove UI :)


## How

- `reader::ArduReader` reads the ardupilot log using the binrw crate, emitting `reader::ArduFrame` enums (either a message definition, a message, or an EOF).
- implementations of `transformers::Transformer` trait convert these `ArduFrame` instances into `transformers::TransformedMessage`
- the `pipeline::process_ardupilot_file` function orchestrates everything, creates MCAP channels and writes messages to them, using `serde_json` schemas and messages
- despite bloated json format, Zstd compression, enabled by default, makes things ok.


## Who

"swarm is us" created this project as a modest contribution to the open-source roboticist community.

We are _not_ affiliated with Foxglove, Ardupilot project, or any of the dependency crates.


## Feature requests, bug reports, etc.

We will entertain them! Create requests on github to start a conversation.

## Contributions, Modifications, Forks

The project is open-source, publicly hosted here: https://github.com/swarmis-us/arducap
We will gladly accept well-written PRs.

### Adding new message conversion, e.g. /ardupilot => /foxglove

The message transformation has been specifically designed to add more conversions.
See transformers.rs -- all it takes is implementing `Transformer` trait and then registering it in the pipeline.rs, alongside other transformers.

## Disclaimer

This software is provided AS IS, without any warranties of any kind. Use it at your own risk.