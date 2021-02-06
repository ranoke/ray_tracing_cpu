#include <iostream>

#include <memory>

#include "utils.h"
#include "vec3.h"


const auto aspect_ratio = 16.0/9.0;
const int image_width = 640;
const int image_height = int(image_width/ aspect_ratio);

uint8_t pixel_buffer[image_width][image_height][3];

const int samples_per_pixel = 100;
const int depth = 40;

struct ray_t;
struct hit_record_t;
struct sphere_t;
struct hittable_t;
struct material_t;

vec3 random_in_unit_sphere() {
  while (true) {
    auto p = vec3::random(-1,1);
    if (p.length_squared() >= 1) continue;
    return p;
  }
}

inline vec3 random_unit_vector()
{
  return unit_vector(random_in_unit_sphere());
}

vec3 reflect(const vec3& v, const vec3& n)
{
  return v - 2*dot(v, n)*n;
}

vec3 refract(const vec3& uv, const vec3& n, double etai_over_etat) {
    auto cos_theta = fmin(dot(-uv, n), 1.0);
    vec3 r_out_perp =  etai_over_etat * (uv + cos_theta*n);
    vec3 r_out_parallel = -sqrt(fabs(1.0 - r_out_perp.length_squared())) * n;
    return r_out_perp + r_out_parallel;
}

struct ray_t
{
  ray_t() {}
  ray_t(const point3& origin, const vec3 dir)
    : origin(origin), dir(dir)
  {}

  point3 at(double t) const
  {
    return origin + t*dir;
  }
  

  point3 origin;
  vec3 dir;
};

struct hit_record_t
{
  point3 p;
  vec3 normal;
  double t;
  bool front_face;
  std::shared_ptr<material_t> mat;

  inline void set_face_normal(const ray_t& r,
                              const vec3& outward_normal)
  {
    front_face = dot(r.dir, outward_normal) < 0;
    normal = front_face ? outward_normal : -outward_normal;
  }
};

struct hittable_t
{
  virtual bool hit(const ray_t& r,
                  double t_min,
                  double t_max,
                  hit_record_t& rec) = 0;
};

struct sphere_t : public hittable_t
{
  sphere_t(){}
  sphere_t(point3 origin, 
           double r,
           std::shared_ptr<material_t> mat)
  : origin(origin), radius(r), mat(mat) {}

  virtual bool hit(const ray_t& r,
                  double t_min,
                  double t_max,
                  hit_record_t& rec) override;


  point3 origin;
  double radius;
  std::shared_ptr<material_t> mat;
};

struct material_t
{
  virtual bool scatter(const ray_t& r_in,
                       const hit_record_t& rec,
                       color& attenuation,
                       ray_t& scattered
                      ) const = 0;
};

struct mat_lambertian_t : public material_t
{
  mat_lambertian_t(const color& a)
    : albedo(a) {}

  virtual bool scatter(const ray_t& r_in,
                       const hit_record_t& rec,
                       color& attenuation,
                       ray_t& scattered
                      ) const override
  {
    auto scattered_dir = rec.normal 
      + random_unit_vector();

    if(scattered_dir.near_zero())
      scattered_dir = rec.normal;

    scattered = ray_t(rec.p, scattered_dir);
    attenuation = albedo;
    return true;
  };

  color albedo;

};

struct mat_metal_t : public material_t
{
  mat_metal_t(const color& a, double f)
    : albedo(a), fuzz(f < 1 ? f : 1) {}

  virtual bool scatter(const ray_t& r_in,
                       const hit_record_t& rec,
                       color& attenuation,
                       ray_t& scattered
                      ) const override
  {
    vec3 reflected = 
      reflect(unit_vector(r_in.dir), rec.normal);
    scattered = ray_t(rec.p, reflected 
        + fuzz*random_in_unit_sphere());
    attenuation = albedo;
    return (dot(scattered.dir, rec.normal) > 0);
  };

  color albedo;
  double fuzz;
};

struct mat_dielectric_t : public material_t
{
  mat_dielectric_t(double index_of_refraction)
    : ir(index_of_refraction) {}

  virtual bool scatter(const ray_t& r_in,
                       const hit_record_t& rec,
                       color& attenuation,
                       ray_t& scattered
                      ) const override
  {
    attenuation = color(1.0, 1.0, 1.0);
double refraction_ratio = rec.front_face ? (1.0/ir) : ir;

vec3 unit_direction = unit_vector(r_in.dir);
double cos_theta = fmin(dot(-unit_direction, rec.normal), 1.0);
double sin_theta = sqrt(1.0 - cos_theta*cos_theta);

bool cannot_refract = refraction_ratio * sin_theta > 1.0;
vec3 direction;

if (cannot_refract)
direction = reflect(unit_direction, rec.normal);
else
direction = refract(unit_direction, rec.normal, refraction_ratio);

scattered = ray_t(rec.p, direction);
return true;
      }
  
  double ir;
};

bool sphere_t::hit(const ray_t& r,
                  double t_min,
                  double t_max,
                  hit_record_t& rec)
{
  vec3 oc = r.origin - origin;
  auto a = dot(r.dir, r.dir);
  auto b = 2.0 * dot(oc, r.dir);
  auto c = dot(oc, oc) - radius*radius;
  auto disc = b*b - 4*a*c;
  if(disc < 0) return false;
  auto sqrtd = sqrt(disc);

  auto root = (-b -sqrtd) / (2*a);
  if(root < t_min || t_max < root)
  {
    root = (-b + sqrtd) / (2*a);
    if(root < t_min || t_max < root)
      return false;
  }

  rec.t = root;
  rec.p = r.at(rec.t);
  rec.normal = (rec.p - origin) / radius;
  rec.mat = mat;
  return true;
}

struct hittable_list_t : public hittable_t
{
  hittable_list_t() {}

  void add(std::shared_ptr<hittable_t> obj) { objects.push_back(obj); }

  virtual bool hit(const ray_t& r,
                  double t_min,
                  double t_max,
                  hit_record_t& rec) override;

  std::vector<std::shared_ptr<hittable_t>> objects;

};

bool hittable_list_t::hit(
    const ray_t& r,
    double t_min,
    double t_max,
    hit_record_t& rec
) 
{
  hit_record_t temp_rec;
  bool hit_anything = false;
  auto closest_so_far = t_max;

  for (const auto& object : objects) {
    if (object->hit(r, t_min, closest_so_far, temp_rec)) {
      hit_anything = true;
      closest_so_far = temp_rec.t;
      rec = temp_rec;
    }
  }

  return hit_anything;
}

color ray_color(const ray_t& r,
               hittable_t& world, int depth)
{
  hit_record_t rec;
  if (depth <= 0)
    return color(0,0,0);

  if (world.hit(r, 0.001, infinity, rec)) {
    ray_t scattered;
    color attenuation;
    if(rec.mat->scatter(r, rec, attenuation, scattered))
      return attenuation * ray_color(scattered, world, depth-1);
    return color(0,0,0);
  }

  vec3 unit_direction = unit_vector(r.dir);
  auto t = 0.5*(unit_direction.y() + 1.0);
  return (1.0-t)*color(1.0, 1.0, 1.0) + t*color(0.5, 0.7, 1.0);
}



void write_color( std::ostream& out,
                  color pixel_color,
                  int x,
                  int y)
{
  auto r = pixel_color.x();
  auto g = pixel_color.y();
  auto b = pixel_color.z();

  auto scale = 1.0 / samples_per_pixel;
  r = sqrt(scale * r);
  g = sqrt(scale * g);
  b = sqrt(scale * b);

  out << static_cast<int>(256 * clamp(r, 0.0, 0.999)) << ' '
      << static_cast<int>(256 * clamp(g, 0.0, 0.999)) << ' '
      << static_cast<int>(256 * clamp(b, 0.0, 0.999)) << '\n';
}




int main()
{
  if(!glfwInit())
    return -1;
  
  auto material_ground = 
    std::make_shared<mat_lambertian_t>(
        color(0.25, 0.25, 0.25));
  /*auto material_center = 
    std::make_shared<mat_lambertian_t>(
        color(0.7, 0.3, 0.3));
  auto material_left   = 
    std::make_shared<mat_metal_t>(
        color(0.8, 0.8, 0.8), 0.3);*/
  auto material_center = 
    std::make_shared<mat_dielectric_t>(0);
  auto material_left = 
    std::make_shared<mat_dielectric_t>(-1.5);
  auto material_right  = 
    std::make_shared<mat_metal_t>(
        color(0.8, 0.6, 0.2), 0);

  hittable_list_t world;
  world.add(std::make_shared<sphere_t>(
    point3(-5,2.5, -4), 5, material_right)
  );
  world.add(std::make_shared<sphere_t>(
    point3(2,0, -3), 0.5, material_ground)
  );
 world.add(std::make_shared<sphere_t>(
    point3(-4,0, -5), 0.5, material_ground)
  );

  world.add(std::make_shared<sphere_t>( 
    point3(0, -100.5, 0), 100, material_ground)
  );
  world.add(std::make_shared<sphere_t>( 
    point3(1, 0, -1), 0.5, material_center)
  );

  for(int i = 0; i < 20; i++)
  {
    auto mat = 
      std::make_shared<mat_lambertian_t>(
          color(rand_double(), rand_double(), rand_double()));
    world.add(std::make_shared<sphere_t>( 
      point3(
        rand_double(-10, 10), 
        0, 
        rand_double(-10, 10)), 0.5, mat));

  }

  
  auto viewport_height = 2.0;
  auto viewport_width = aspect_ratio * viewport_height;
  auto focal_length = 1.0;

  auto origin = point3(0,2,3);
  auto horizontal = vec3(viewport_width, 0, 0);
  auto vertical = vec3(0, viewport_height, 0);
  auto lower_left_corner = origin - horizontal/2
    - vertical/2 - vec3(0,0, focal_length);


  std::cout << "P3\n" << image_width 
    << ' ' << image_height << "\n255\n";
  for(int y = image_height-1; y >= 0; --y)
  {
    std::cerr << "\rScanlines remaining: " << y
      << ' ' << std::flush;
    for(int x = 0; x < image_width; ++x)
    {
      color pixel_color(0,0,0);
      for(int s = 0; s < samples_per_pixel; s++)
      {
        auto u = (x + rand_double())
          / (image_width-1);
        auto v = (y + rand_double())
          / (image_height-1);

        ray_t r(origin, 
            lower_left_corner
            + u*horizontal + v*vertical - origin);
        pixel_color += ray_color(r, world, depth);
      }

      write_color(std::cout, pixel_color, x, y);
      
    }
  }



  return 0;
}
