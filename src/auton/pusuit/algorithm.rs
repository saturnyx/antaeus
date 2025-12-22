use std::f64::{EPSILON, MAX};

use log::{error, info};

use super::geo;

fn point_in_circle(p: &geo::Point, cir: &geo::Circle) -> bool {
    let h = cir.x;
    let k = cir.y;
    let x = p.x;
    let y = p.y;
    let a = (x - h).powi(2);
    let b = (y - k).powi(2);
    let c = a + b - cir.r.powi(2);
    c < 0.0
}

fn line_circ_intersect(line: geo::Line, cir: &geo::Circle) -> Vec<geo::Point> {
    let p1 = line.point1;
    let p2 = line.point2;
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;
    let fx = p1.x - cir.x;
    let fy = p1.y - cir.y;
    let a = dx * dx + dy * dy;
    let b = 2.0 * (fx * dx + fy * dy);
    let c = fx * fx + fy * fy - cir.r * cir.r;
    let d = b * b - 4.0 * a * c;
    if d < 0.0 {
        return Vec::new();
    }
    let sqrt_d = d.sqrt();
    let t1 = (-b - sqrt_d) / (2.0 * a);
    let t2 = (-b + sqrt_d) / (2.0 * a);
    let mut intersections = Vec::new();

    if (0.0..=1.0).contains(&t1) {
        intersections.push(geo::Point::new(p1.x + t1 * dx, p1.y + t1 * dy));
    }

    if (0.0..=1.0).contains(&t2) && (t2 - t1).abs() > EPSILON {
        intersections.push(geo::Point::new(p1.x + t2 * dx, p1.y + t2 * dy));
    }

    intersections
}

fn path_circ_intersect(path: &geo::Path, cir: geo::Circle) -> Vec<geo::Point> {
    let lines = path.get_lines();
    let mut intersects: Vec<geo::Point> = Vec::new();
    for line in lines {
        intersects.append(&mut line_circ_intersect(line, &cir));
    }
    intersects
}

fn prox_point_on_line(line: geo::Line, point: geo::Point) -> (geo::Point, f64) {
    let line_vec_x = line.point2.x - line.point1.x;
    let line_vec_y = line.point2.y - line.point1.y;
    let point_vec_x = point.x - line.point1.x;
    let point_vec_y = point.y - line.point1.y;
    let dot_product = point_vec_x * line_vec_x + point_vec_y * line_vec_y;
    let line_length_squared = line_vec_x * line_vec_x + line_vec_y * line_vec_y;
    if line_length_squared == 0.0 {
        let distance =
            ((point.x - line.point1.x).powi(2) + (point.y - line.point1.y).powi(2)).sqrt();
        return (line.point1, distance);
    }

    let t = (dot_product / line_length_squared).clamp(0.0, 1.0);
    let closest_point =
        geo::Point::new(line.point1.x + t * line_vec_x, line.point1.y + t * line_vec_y);
    let distance =
        ((point.x - closest_point.x).powi(2) + (point.y - closest_point.y).powi(2)).sqrt();
    (closest_point, distance)
}

fn prox_point_on_path(path: &geo::Path, point: geo::Point) -> (geo::Point, f64) {
    let mut prox_pt = geo::Point::new(0.0, 0.0);
    let mut dist = MAX;
    for line in path.get_lines() {
        let (pt, d) = prox_point_on_line(line, point);
        if d < dist {
            prox_pt = pt;
            dist = d;
        }
    }
    (prox_pt, dist)
}

fn get_candidates(path: &geo::Path, cir: geo::Circle) -> Vec<geo::Point> {
    let mut c: Vec<geo::Point> = Vec::new();
    for p in path.waypoints.clone() {
        if point_in_circle(&p, &cir) {
            c.push(p);
        }
    }
    c.append(&mut path_circ_intersect(path, cir));
    let (prox_pt, _) = prox_point_on_path(path, geo::Point::new(cir.x, cir.y));
    c.push(prox_pt);
    c
}

fn point_t_on_line(point: geo::Point, line: geo::Line) -> Option<f64> {
    let p1 = line.point1;
    let p2 = line.point2;
    let dx = p2.x - p1.x;
    let dy = p2.y - p1.y;

    let t = if dx.abs() > dy.abs() {
        (point.x - p1.x) / dx
    } else if dy.abs() > EPSILON {
        (point.y - p1.y) / dy
    } else {
        let dist = ((point.x - p1.x).powi(2) + (point.y - p1.y).powi(2)).sqrt();
        return if dist < EPSILON { Some(0.0) } else { None };
    };

    let computed_x = p1.x + t * dx;
    let computed_y = p1.y + t * dy;

    let dist = ((point.x - computed_x).powi(2) + (point.y - computed_y).powi(2)).sqrt();

    if dist < EPSILON && t >= -EPSILON && t <= 1.0 + EPSILON {
        Some(t.clamp(0.0, 1.0))
    } else {
        None
    }
}

fn get_t(point: geo::Point, path: &geo::Path) -> Option<f64> {
    let lines = path.get_lines();
    for (index, line) in lines.iter().enumerate() {
        if let Some(t) = point_t_on_line(point, *line) {
            return Some(index as f64 + t);
        }
    }
    None
}

fn get_target(candidates: Vec<geo::Point>, path: geo::Path) -> geo::Point {
    let mut target: geo::Point = geo::Point::new(0.0, 0.0);
    let mut target_t = -MAX;
    for point in candidates {
        let t = get_t(point, &path).unwrap_or_else(|| {
            error!("Could not find `t` for point ({},{}) on path", point.x, point.y);
            info!("Ignoring point point ({},{})", point.x, point.y);
            -MAX
        });
        if t > target_t {
            target = point;
            target_t = t;
        }
    }
    target
}

pub fn pp_target(path: geo::Path, cir: geo::Circle) -> geo::Point {
    let candidates = get_candidates(&path, cir);
    get_target(candidates, path)
}
