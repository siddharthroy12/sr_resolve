# sr_resolve.h

A simple single header C libary for AABB Collision detection and resolution

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
```