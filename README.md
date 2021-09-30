# sr_resolve.h

A simple single header C/C++ Library for AABB Collision detection and resolution

![](./demo.gif)

## Structs

```c
// Rectangle
typedef struct sr_rec {
	float x;
	float y;
	float width;
	float height;
} sr_rec;

// 2D Vector
typedef struct sr_vec2 {
	float x;
	float y;
} sr_vec2;

// 2D Ray
typedef struct sr_vec2 {
  sr_vec2 position;
  sr_vec2 direction;
} sr_vec2;
```

## Function
```c
bool sr_check_rec_vs_rec_collision(sr_rec rec1, sr_rec rec2);
bool sr_check_ray_vs_rec_collision(const sr_ray2 ray, const sr_rec target, sr_vec2 *contact_point, sr_vec2 *contact_normal, float *t_hit_near);
bool sr_dynamic_rect_vs_rect(const sr_rec in, const sr_rec target, sr_vec2 vel, sr_vec2 *contact_point, sr_vec2 *contact_normal, float *contact_time, float delta)
// The function you will use in your game
void sr_move_and_slide(sr_rec *obstacles, int obstacles_length, sr_vec2 hitbox, sr_vec2 *vel, sr_vec2 *pos , float delta);
```

## Example
Here is a demo
https://github.com/siddharthroy12/sr_resolve_example