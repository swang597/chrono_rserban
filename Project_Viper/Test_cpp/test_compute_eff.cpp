#include <torch/torch.h>
#include <iostream>
#include <vector>

using namespace torch;

Tensor fill_between_edges_vectorized(const Tensor& mask) {
    if (!mask.any().item<bool>()) {
        return mask;
    }

    auto cummax_top_bottom = mask.cummax(0, false).values;
    auto cummax_bottom_top = mask.flip(0).cummax(0, false).values.flip(0);
    auto filled_mask = cummax_top_bottom & cummax_bottom_top;

    return filled_mask;
}

std::pair<Tensor, Tensor> find_wheel_contact(const Tensor& height_map, const Tensor& wheel_center, 
                                             float radius, float width, float grid_size, 
                                             float threshold, int cx_idx, int cy_idx) {
    int64_t nx = height_map.size(-2);
    int64_t ny = height_map.size(-1);
    float cx = cx_idx * grid_size;
    float cy = cy_idx * grid_size;
    float cz = wheel_center[2].item<float>();

    Tensor x = arange(0, nx, torch::kFloat32) * grid_size;
    Tensor y = arange(0, ny, torch::kFloat32) * grid_size;
    Tensor xv = x.unsqueeze(1).expand({nx, ny});
    Tensor yv = y.unsqueeze(0).expand({nx, ny});

    Tensor distance_x = torch::abs(xv - cx);
    Tensor distance_y = torch::abs(yv - cy);

    Tensor within_width = distance_y < (width / 2);
    Tensor within_radius = distance_x < radius;

    Tensor wheel_surface_height = cz - torch::sqrt(radius * radius - distance_x.pow(2)).clamp_min(0);
    Tensor nearby_current_height = torch::abs(wheel_surface_height - height_map[0][0]) < threshold;

    Tensor mask = within_width & within_radius & nearby_current_height;

    Tensor edge_kernel = torch::tensor({{{1, 1, 1},
                                         {1, -8, 1},
                                         {1, 1, 1}}}, torch::kFloat32).unsqueeze(0).unsqueeze(0);

    Tensor mask_float = mask.to(torch::kFloat32).unsqueeze(0).unsqueeze(0);
    Tensor edges = torch::conv2d(mask_float, edge_kernel, {}, 1, 1);
    Tensor edge_mask = edges.squeeze().to(torch::kBool);

    return std::make_pair(mask, edge_mask);
}

Tensor compute_dHM_eff(Tensor height_map, const std::vector<float>& wheel_center, float wheel_radius, 
                       float wheel_width, float terrain_grid, float threshold_contact, 
                       int cx_idx, int cy_idx) {
    if (height_map.dim() != 4) {
        height_map = height_map.unsqueeze(0).unsqueeze(0).to(torch::kFloat32);
    }

    auto wheel_center_ts = torch::tensor(wheel_center, torch::kFloat32);
    
    auto [mask, edge_mask] = find_wheel_contact(height_map, wheel_center_ts, wheel_radius, wheel_width, 
                                                terrain_grid, threshold_contact, cx_idx, cy_idx);
    if (!mask.any().item<bool>()) {
        return Tensor();
    }

    auto filled_mask = fill_between_edges_vectorized(mask);

    int64_t nx = height_map.size(-2);
    int64_t ny = height_map.size(-1);
    Tensor data_HM_init = torch::zeros({nx, ny}, torch::kFloat32);
    data_HM_init.slice(1, 0, ny) = height_map[0][0].slice(1, 0, ny).unsqueeze(1);

    Tensor data_HM_geo_ts = data_HM_init.unsqueeze(0).unsqueeze(0).to(torch::kFloat32);

    // Placeholder for apply_wheel_sinkage function
    // apply_wheel_sinkage(data_HM_geo_ts, wheel_center_ts, wheel_radius, wheel_width, terrain_grid, cx_idx, cy_idx);

    Tensor dHM_eff = torch::zeros({nx, ny}, torch::kFloat32);
    Tensor data_HM_geo_np = data_HM_geo_ts[0][0];
    dHM_eff.index_put_({filled_mask}, data_HM_init.index({filled_mask}) - data_HM_geo_np.index({filled_mask}));

    return dHM_eff;
}

int main() {
    // Example usage
    std::vector<float> wheel_center = {10.0, 20.0, 30.0};
    float wheel_radius = 5.0;
    float wheel_width = 2.0;
    float terrain_grid = 0.1;
    float threshold_contact = 0.05;
    int cx_idx = 10;
    int cy_idx = 20;

    // Placeholder height_map tensor
    Tensor height_map = torch::rand({1, 1, 100, 100}, torch::kFloat32);

    Tensor result = compute_dHM_eff(height_map, wheel_center, wheel_radius, wheel_width, 
                                    terrain_grid, threshold_contact, cx_idx, cy_idx);
    std::cout << result << std::endl;

    return 0;
}
