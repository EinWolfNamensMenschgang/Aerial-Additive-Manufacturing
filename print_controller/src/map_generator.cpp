#include <vector>
#include <fstream>
#include <yaml-cpp/yaml.h> // You might need to install yaml-cpp library

class map {
public:
    void setRowsAndCols(int rows, int cols) {
        rows_ = rows;
        cols_ = cols;
        return;
    }

    void setOffset(double x, double y, double z) {
        offset_x_ = x;
        offset_y_ = y;
        offset_z_ = z;
        return;
    }

    void setMarkerSpace(double vertical, double horizontal) {
        space_between_vertical_ = vertical;
        space_between_horizontal_ = horizontal;
        return;
    }

    void calculateMarkerPositions() {
        for (int i = 0; i < rows_; i++) {
            for (int j = 0; j < cols_; j++) {
                MarkerPose markerPos;
                markerPos.x = offset_x_ + j * space_between_horizontal_;
                markerPos.y = offset_y_;
                markerPos.z = offset_z_ - i * space_between_vertical_;
                markerPos.roll = 1.5707963267948966;
                markerPos.pitch = 2.2204460492503126e-16;
                markerPos.yaw = 0.0;
                marker_positions_.push_back(markerPos);
            }
        }
    }

    void saveMarkerPositionsToYAML(const std::string& filename) {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "marker_length" << YAML::Value << marker_length_;
        out << YAML::Key << "markers" << YAML::Value << YAML::BeginSeq;
        for (size_t i = 0; i < marker_positions_.size(); ++i) {
            out << YAML::BeginMap;
            out << YAML::Key << "id" << YAML::Value << static_cast<int>(i);
            out << YAML::Key << "u" << YAML::Value << 1;
            out << YAML::Key << "f" << YAML::Value << 1;
            out << YAML::Key << "xyz" << YAML::Flow << YAML::BeginSeq << marker_positions_[i].x << marker_positions_[i].y << marker_positions_[i].z << YAML::EndSeq;
            out << YAML::Key << "rpy" << YAML::Flow << YAML::BeginSeq << marker_positions_[i].roll << marker_positions_[i].pitch << marker_positions_[i].yaw << YAML::EndSeq;
            out << YAML::EndMap;
        }
        out << YAML::EndSeq;
        out << YAML::EndMap;

        std::ofstream fout(filename);
        fout << out.c_str();
    }

private:
    struct MarkerPose {
        double x, y, z;
        double roll, pitch, yaw;
    };

    int rows_;
    int cols_;
    double offset_x_;
    double offset_y_;
    double offset_z_;
    double space_between_horizontal_;
    double space_between_vertical_;
    std::vector<MarkerPose> marker_positions_;
    double marker_length_ = 0.092;
};

int main() {
    map myMap;
    myMap.setRowsAndCols(2, 5);
    myMap.setOffset(-0.42, 1.5, 0.3);
    myMap.setMarkerSpace(0.296, 0.21);
    myMap.calculateMarkerPositions();
    myMap.saveMarkerPositionsToYAML("/home/ws/src/src/Aerial-Additive-Manufacturing/print_controller/map/output_fiducial_map.yaml");

    return 0;
}
