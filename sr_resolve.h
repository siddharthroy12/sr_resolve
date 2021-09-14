// A simple AABB Collision resolver that will work for most games
// Author - siddharthroy12/@siddharthroy12

#ifndef SR_RESOLVE_H
#define SR_RESOLVE_H
#include <math.h>

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

// Get the length of sr_vec2
float sr_vec2_length(sr_vec2 v) {
	float result = sqrtf((v.x*v.x) + (v.y*v.y));
    return result;
}

// Scale sr_vec2
static sr_vec2 sr_vec2_scale(sr_vec2 v, float scale) {
    sr_vec2 result = { v.x*scale, v.y*scale };
    return result;
}

// Normalize sr_vec2
static sr_vec2 sr_vec2_normalize(sr_vec2 v) {
    sr_vec2 result = sr_vec2_scale(v, 1 / sr_vec2_length(v));
    return result;
}

// Substract sr_vec2
static sr_vec2 sr_vector2_sub(sr_vec2 v1, sr_vec2 v2) {
    sr_vec2 result = { v1.x - v2.x, v1.y - v2.y };
    return result;
}

// Check collision between two sr_rec
static bool sr_check_collision_recs(sr_rec rec1, sr_rec rec2) {
    bool collision = false;

    if (
		(rec1.x < (rec2.x + rec2.width) && (rec1.x + rec1.width) > rec2.x) &&
        (rec1.y < (rec2.y + rec2.height) && (rec1.y + rec1.height) > rec2.y)
	) {
		collision = true;
	} 

    return collision;
}

// Get the area of the collision
static sr_rec sr_get_collision_rec(sr_rec rec1, sr_rec rec2) {
    sr_rec rec = { 0, 0, 0, 0 };

    if (sr_check_collision_recs(rec1, rec2)) {
        float dxx = fabs(rec1.x - rec2.x);
        float dyy = fabs(rec1.y - rec2.y);

        if (rec1.x <= rec2.x) {
            if (rec1.y <= rec2.y) {
                rec.x = rec2.x;
                rec.y = rec2.y;
                rec.width = rec1.width - dxx;
                rec.height = rec1.height - dyy;
            } else {
                rec.x = rec2.x;
                rec.y = rec1.y;
                rec.width = rec1.width - dxx;
                rec.height = rec2.height - dyy;
            }
        } else {
            if (rec1.y <= rec2.y) {
                rec.x = rec1.x;
                rec.y = rec2.y;
                rec.width = rec2.width - dxx;
                rec.height = rec1.height - dyy;
            } else {
                rec.x = rec1.x;
                rec.y = rec1.y;
                rec.width = rec2.width - dxx;
                rec.height = rec2.height - dyy;
            }
        }

        if (rec1.width > rec2.width) {
            if (rec.width >= rec2.width) rec.width = rec2.width;
        }
        else {
            if (rec.width >= rec1.width) rec.width = rec1.width;
        }

        if (rec1.height > rec2.height) {
            if (rec.height >= rec2.height) rec.height = rec2.height;
        } else {
           if (rec.height >= rec1.height) rec.height = rec1.height;
        }
    }

    return rec;
}

// Get the vector between the center of two rects
static sr_vec2 sr_get_center_distance_vec(sr_rec rec1, sr_rec rec2) {
    sr_vec2 rec1_center = (sr_vec2){
        .x = rec1.x + (rec1.width/2),
        .y = rec1.y + (rec1.height/2)
    };

    sr_vec2 rec2_center = (sr_vec2) {
        .x = rec2.x + (rec2.width/2),
        .y = rec2.y + (rec2.height/2)
    };

    return sr_vec2_normalize(sr_vector2_sub(rec2_center, rec1_center));
}

//Takes two rects and resolve the position of the second one
static sr_rec sr_resolver_rects_collision(sr_rec static_rec, sr_rec rec_to_move) {
	if (!sr_check_collision_recs(static_rec, rec_to_move)) {
		return rec_to_move;
	} else {
		sr_rec collision_rec = sr_get_collision_rec(static_rec, rec_to_move);
		sr_vec2 center_distance_vec = sr_get_center_distance_vec(static_rec, rec_to_move);
		sr_rec rec_copy = rec_to_move;

		if (fabs(center_distance_vec.x) > fabs(center_distance_vec.y)) {
            if (center_distance_vec.x > 0) {
                rec_copy.x = rec_copy.x + collision_rec.width;
            } else {
                rec_copy.x = rec_copy.x - collision_rec.width;
            }
        } else {
            if (center_distance_vec.y > 0) {
                rec_copy.y = rec_copy.y + collision_rec.height;
            } else {
                rec_copy.y = rec_copy.y - collision_rec.height;
            }
        }

		return rec_copy;
	}
}

// Move and collide rect with rects
static sr_rec sr_move_and_collide(sr_rec *static_recs, int number_of_recs, sr_rec rec_to_move, sr_vec2 velocity) {
    sr_rec before_move = rec_to_move;

    rec_to_move.x += velocity.x;
    rec_to_move.y += velocity.y;

    sr_rec result = rec_to_move;

    for (int i = 0; i < number_of_recs; i++) {
        result = sr_resolver_rects_collision(static_recs[i], result);
    }

    if (result.x == before_move.x && result.y == before_move.y) {
        result.x += velocity.x;
        result.y += velocity.y;
        for (int i = (number_of_recs-1); i > -1; i--) {
            result = sr_resolver_rects_collision(static_recs[i], result);
        }
    }

    return result;
}

#endif