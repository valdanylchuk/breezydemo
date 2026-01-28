# BreezyBox shell demo for ESP32-S3

This is a demo for how you can turn an ESP32-S3 microcontroller into a **tiny instant-on PC** with its own shell, editor, compiler, and online apps installer. Something like Raspberry Pi, minus the overhead of a full server/desktop grade OS. I think ESP32 is underrated in hobby maker community for this PC-like use case. This demo uses [BreezyBox](https://github.com/valdanylchuk/breezybox), my mini-shell ESP-IDF component.

## That sounds too good to be true. What is BreezyBox anyway?

First of all, seeing is believing:

[insert demo video]

It started as a "cyberdeck" style crafting project. Then I got carried away with the software part. I chose ESP32-S3 for the base platform. It has the nostalgic appeal of the DOS era PCs, with similar resources, and elbow-deep-in-bytes coding experience, plus modern wireless comms.

> ESP32-S3 can do everything those PCs did and more, but that is inconvenient out of the box, because that is not the commercial use case it is positioned for. It also forces away the code bloat. If you are like me, and love small elegant things, and technology that punches way above its weight, you ought to try it!

So anyway, I decided to try and package some key missing parts: a basic vterm, the current working directory (CWD) tracking, a few familiar UNIX-like commands, and an app installer. Believe it or not, the rest is already there in ESP-IDF components, including the elf_loader with dynamic linking.

The result is called "BreezyBox", by analogy with the BusyBox commands suite. The name is just a light joke, it is not meant to be a full clone. You can import it with one command in your ESP-IDF project, and if you have some stdio going, even at "Hello World" level, it should mostly just work. I call it a "mini shell", a naïve user might call it an OS (it is not, it runs on FreeRTOS), and you can also call it the userland layer.

## This repo is just one possible example

The BreezyBox component leaves the display and other board configuration details to the user's firmware project, providing mainly the vterm/vfs features, and some shell commands. This particular example/demo project supports only one specific dev board: [Waveshare ESP32-S3-Touch-LCD-7B](https://www.waveshare.com/esp32-s3-lcd-7b.htm) (no affiliation). But you can see how all the parts connect, and adapt it to your display/board, or just copy some code snippets from here.

## How to Use

I suggest just fork it, clone it, and try to make it work on your board. Mine was about 40€; you can start with some random $10 two inch LCD S3 dev board if you like. Hint: LVGL text label control is the easiest path to stdout on LCD that works almost everywhere. You can also start with a headless board over USB console, that takes zero code, and gives you free ANSI codes in standard IDF Monitor in VSCode (or in Tabby).

You do not have to write your own font renderer like I did here; that was just to push past 30 FPS on a display slightly too large for this chip.

## License

This is free software under [MIT License](LICENSE).

## Contributing: Help Wanted!

The best help is currently more testing beyond "works on my computer", more shared examples and fun use cases:

1. More ELF apps – see the examples at my [breezyapps repo](https://github.com/valdanylchuk/breezyapps), they are super easy to follow. Even a carefully written stdlib C program with no platform-specific bits may work sometimes, also with some ANSI codes. But be sure to verify on the actual ESP32-S3: the memory is tight, the larger PSRAM requires alignment, and there are other limits and quirks. You can publish and install the apps using your own repo.

2. More full example firmware repositories: for different boards, with different styles. Maybe you provide the basic LVGL text label example on some popular board. Maybe you prefer C++ to plain C. Maybe you embrace the GUI. Maybe you port some retro games. Maybe you even make it work on P4, or C6 (RISC-V, a completely different CPU). Maybe you attach some cool gadgets to it. Maybe you build an extra cool cyberdeck case. Or maybe you reproduce the exact same thing, and just share your setup experience and hands-on impressions.

It would be so cool to see more people using BreezyBox, and to have more ready-to-clone examples for everyone!

Have fun!