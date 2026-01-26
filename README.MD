# GS.Phy2D

Since [APE](https://github.com/VincentGsell/APE) repo, I'll trying to work on a more optimized 2d physics stuff, here is the WIP result.
-> Architecture work + code layout to make it run quite fast (spatial hashes, records, pointers lists and so on.)

I wrote the Pascal version in a pretty chill way, really from start to finish, taking inspiration from [APE](https://github.com/VincentGsell/APE) of course, 
but also from the work done on the [2D constraint project](https://github.com/VincentGsell/2d-Constraints-Demos)  (also written in a relaxed style 🙂).

While writing the spatial hash, I did a lot of research and, with help from AI (Claude), that turned out to be a key factor in several parts of the code.

I also started a Rust course a few months ago, which I find quite fascinating. So I began translating the Pascal project into Rust — heavily assisted by AI at first, but less and less over time 🙂

So there’s some documentation (AI-assisted, but reviewed and overseen by me) that includes a small comparative analysis.

## History : 

2026-01-26 - Rust version - Docs.

![Alt text](/assets/rust-gsPhy2D-1.png "rustImage1")
![Alt text](/assets/rust-gsPhy2D-2.png "rustImage2")

2026-01-21 - Compound constraint.

![Alt text](/assets/gsPhy2DDemo1CompoundConstraint.png "Compound")


2026-01-11 - change GUI - AABB fynamic - OBB R&D

2026-01-09 - AABB Static - Circle shape - Optimisation

2025-12-10 - Early stage.

![Alt text](/assets/gsPhy2DDemo1WaterLike202601.gif?raw=true "fluids animation")
![Alt text](/assets/gsPhy2DDemo1WaterLike20260111-1.png "remind us fluids")
![Alt text](/assets/gsPhy2DDemo1WaterLike20260111-2.png "Wave")

## Older version 

![Alt text](/assets/gsPhy2DDemo1WaterLike.gif?raw=true "fluids animation")
![Alt text](/assets/gsPhy2DDemo1WaterLike2Bigger_1.png "remind us fluids")
