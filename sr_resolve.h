// A simple AABB Collision resolver that will work for most games
// Author - siddharthroy12/@siddharthroy12

#ifndef SR_RESOLVE_H
#define SR_RESOLVE_H
#include <math.h>
#include <stdio.h>

#define max(x, y) (x > y ? x : y)
#define min(x, y) (x > y ? y : x)

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

typedef struct sr_ray2 {
    sr_vec2 position;
    sr_vec2 direction;
} sr_ray2;

typedef struct sr_sort_pair {
    int index;
    float time;
} sr_sort_pair;

void swap_float(float *vec1, float *vec2) {
  	float temp = *vec1;
  	*vec1 = *vec2;
  	*vec2 = temp;
}

// Get the length of a vector
float sr_vec2_length(sr_vec2 v) {
	float result = sqrtf((v.x*v.x) + (v.y*v.y));
  	return result;
}

// Scale a vector by a given float value
static sr_vec2 sr_vec2_scale(sr_vec2 v, float scale) {
  	return (sr_vec2){ v.x*scale, v.y*scale };
}

// Divide two vectors
static sr_vec2 sr_vec2_divide(sr_vec2 v1, sr_vec2 v2) {
  	return (sr_vec2){ v1.x/v2.x, v1.y/v2.y };
}

// Multiply two vectors
static sr_vec2 sr_vec2_multiply(sr_vec2 v1, sr_vec2 v2) {
  	return (sr_vec2){ v1.x*v2.x, v1.y*v2.y };
}

// Normalize a vector
static sr_vec2 sr_vec2_normalize(sr_vec2 v) {
  	return sr_vec2_scale(v, 1 / sr_vec2_length(v));
}

// Substract two vectors
static sr_vec2 sr_vector2_sub(sr_vec2 v1, sr_vec2 v2) {
  	return (sr_vec2){ v1.x - v2.x, v1.y - v2.y };
}

// Add two vectors
static sr_vec2 sr_vector2_add(sr_vec2 v1, sr_vec2 v2) {
  	return (sr_vec2){ v1.x + v2.x, v1.y + v2.y };
}

// Check collision between two rectangles
static bool sr_check_rec_vs_rec_collision(sr_rec rec1, sr_rec rec2) {
    if (
		(rec1.x < (rec2.x + rec2.width) && (rec1.x + rec1.width) > rec2.x) &&
        (rec1.y < (rec2.y + rec2.height) && (rec1.y + rec1.height) > rec2.y)
	) {
		return true;
	} else {
        return false;
    }
}

// Check collision between a ray and a rectangle
static bool sr_check_ray_vs_rec_collision(const sr_ray2 ray, const sr_rec target, sr_vec2 *contact_point, sr_vec2 *contact_normal, float *t_hit_near) {
    // Cache division
    sr_vec2 indiv = sr_vec2_divide((sr_vec2){1.0f, 1.0f}, ray.direction);

    // Calculate intersections with rectangle bounding axes
    sr_vec2 t_near = sr_vec2_multiply(
        sr_vector2_sub((sr_vec2){ target.x, target.y }, ray.position),
        indiv
    );
    sr_vec2 t_far = sr_vec2_multiply(
        sr_vector2_sub(
            sr_vector2_add((sr_vec2){target.x, target.y}, (sr_vec2){target.width, target.height}),
            ray.position
        ),
        indiv
    );

    // Check for nan
    if (isnanf(t_far.y) || isnanf(t_far.x)) return false;
	if (isnanf(t_near.y) || isnanf(t_near.x)) return false;

    // Sort distances
    if (t_near.x > t_far.x) swap_float(&t_near.x, &t_far.x);
    if (t_near.y > t_far.y) swap_float(&t_near.y, &t_far.y);

    // Early rejection		
	if (t_near.x > t_far.y || t_near.y > t_far.x) return false;

    // Closest 'time' will be the first contact
    *t_hit_near = max(t_near.x, t_near.y);

    // Furthest 'time' is contact on opposite side of target
    float t_hit_far = min(t_far.x, t_far.y);

    // Reject if ray direction is pointing away from object
    if (t_hit_far < 0) return false;

    // Contact point of collision from parametric line equation
    *contact_point = sr_vector2_add(ray.position, sr_vec2_scale(ray.direction, *t_hit_near));

	if (t_near.x > t_near.y)
		if (ray.direction.x < 0)
			*contact_normal = (sr_vec2){ 1, 0 };
		else
			*contact_normal = (sr_vec2){ -1, 0 };
	else if (t_near.x < t_near.y)
		if (ray.direction.y < 0)
			*contact_normal = (sr_vec2){ 0, 1 };
		else
			*contact_normal = (sr_vec2){ 0, -1 };

    return true;
}

static bool sr_dynamic_rect_vs_rect(const sr_rec in, const sr_rec target, sr_vec2 vel, sr_vec2 *contact_point, sr_vec2 *contact_normal, float *contact_time, float delta) {
    // Check if dynamic rectangle is actually moving - we assume rectangles are NOT in collision to start
    if (vel.x == 0 && vel.y == 0) return false;
    
	// Expand target rectangle by source dimensions
    sr_rec expanded_target;
    expanded_target.x = target.x - (in.width/2);
    expanded_target.y = target.y - (in.height/2);
    expanded_target.width = target.width + in.width;
    expanded_target.height = target.height + in.height;

    if (sr_check_ray_vs_rec_collision(
      (sr_ray2){
        (sr_vec2){
          in.x + (in.width/2),
          in.y + (in.height/2)
        },
        sr_vec2_scale(vel, delta)
      },
      expanded_target,
      contact_point,
      contact_normal,
      contact_time
    )) {
      if (*contact_time <= 1.0f && *contact_time >= -1.0f) return true;
    }

    return false;
}

void sr_sort_indexes(sr_sort_pair *times, int length) {
    sr_sort_pair key;
    int i, j;

    for (i = 1; i < length; i++) {
      key = times[i];
      j = i - 1;

      while(j >= 0 && times[j].time > key.time) {
        times[j+1] = times[j];
        j = j -1;
      }

      times[j + 1] = key;

    }

    // for (int i = 0; i < length; ++i) {
    //   for (int j = i + 1; j < length; ++j) {
    //     if (times[i].time > times[j].time) {
    //       key =  times[i];
    //       times[i] = times[j];
    //       times[j] = key;
		// 		}
		// 	}
    // }
}

static void sr_move_and_slide(sr_rec *obstacles, int obstacles_length, sr_vec2 hitbox, sr_vec2 *vel, sr_vec2 *pos , float delta) {
	sr_sort_pair times[obstacles_length];
	sr_vec2 cp, cn;
	float time = 0;

  	sr_rec hitbox_rec = {
  	  	pos->x - (hitbox.x/2),
  	  	pos->y - (hitbox.y/2),
  	  	hitbox.x,
  	  	hitbox.y
  	};


	for (int i = 0; i < obstacles_length; i++) {
    	sr_dynamic_rect_vs_rect(hitbox_rec, obstacles[i], *vel, &cp, &cn, &time, delta);
    	times[i].index = i;
    	times[i].time = time;
  	}

	sr_sort_indexes(times, obstacles_length);

	for (int i = 0; i < obstacles_length; i++) {  
    	if (sr_dynamic_rect_vs_rect(hitbox_rec, obstacles[times[i].index], *vel, &cp, &cn, &time, delta)) {
    	    pos->x = cp.x;
    	    pos->y = cp.y;

			if (fabs(cn.x)) {
    	    	vel->x = 0;
    	    }

    	    if (fabs(cn.y)) {
    	    	vel->y = 0;
    	    }
    	}
  	}
}
#endif