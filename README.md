# sr_resolve.h

A simple single header C/C++ Library for AABB Collision detection and resolution

![](./demo.gif)

## Structs

```c
typedef struct sr_rec {
	float x;
	float y;
	float width;
	float height;
} sr_rec;

typedef struct sr_vec2 {
	float x;
	float y;
} sr_vec2;
```

## Function
```c
sr_check_collision_recs(sr_rec rec1, sr_rec rec2);
sr_get_collision_rec(sr_rec rec1, sr_rec rec2);
sr_rec sr_resolver_rects_collision(sr_rec static_rec, sr_rec rec_to_move);
sr_rec sr_move_and_collide(sr_rec *static_recs, int number_of_recs, sr_rec rec_to_move, sr_vec2 velocity)
```

## Example
Here is a demo
https://github.com/siddharthroy12/sr_resolve_example