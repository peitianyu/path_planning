#include"grid.h"



Grid::Grid(const Option &p, const std::string &grid_file) : m_option(p)
{
    ReadGridFile(grid_file);
}

bool Grid::VerifyNode(const Point &pos)
{
    if (!IsValid(pos))
        return false;

    if (IsWall(pos))
        return false;

    return true;
}

bool Grid::IsValid(const Point &pos)
{
    if (pos.x < 0 || pos.x >= m_grid.rows())
        return false;

    if (pos.y < 0 || pos.y >= m_grid.cols())
        return false;

    return true;
}

bool Grid::IsWall(const Point &pos)
{
    return m_grid(pos.x, pos.y) == m_option.wall;
}

bool Grid::IsFree(const Point &pos)
{
    return m_grid(pos.x, pos.y) == m_option.free;
}

bool Grid::IsUnknown(const Point &pos)
{
    return m_grid(pos.x, pos.y) == m_option.unknown;
}

void Grid::Visual()
{
    constexpr int grid_size = 20;
    // Create a color map for the grid data
    std::map<int, cv::Scalar> color_map;
    color_map[m_option.wall] = cv::Scalar(0, 0, 0);          // Black for walls
    color_map[m_option.free] = cv::Scalar(255, 255, 255);    // White for free spaces
    color_map[m_option.unknown] = cv::Scalar(128, 128, 128); // Gray for unknown spaces
    color_map[m_option.path] = cv::Scalar(255, 0, 0); // Blue for path spaces

    // Create a image to display the grid data
    cv::Mat image(m_grid.cols() * grid_size, m_grid.rows() * grid_size, CV_8UC3, cv::Scalar(0, 0, 0));

    // Fill the image with the appropriate colors
    for (int i = 0; i < m_grid.rows(); i++)
    {
        for (int j = 0; j < m_grid.cols(); j++)
        {
            cv::Point p1(j * grid_size, (m_grid.rows() - i - 1) * grid_size);
            cv::Point p2((j + 1) * grid_size, (m_grid.rows() - i) * grid_size);
            cv::rectangle(image, p1, p2, color_map[m_grid(j, i)], -1);
        }
    }

    // Draw horizontal and vertical lines on the image
    for (int i = 0; i < m_grid.cols() + 1; i++)
    {
        cv::Point p1(i * grid_size, 0);
        cv::Point p2(i * grid_size, m_grid.rows() * grid_size);
        cv::line(image, p1, p2, cv::Scalar(0, 0, 0), 1);
    }
    
    for (int i = 0; i < m_grid.rows() + 1; i++)
    {
        cv::Point p1(0, i * grid_size);
        cv::Point p2(m_grid.cols() * grid_size, i * grid_size);
        cv::line(image, p1, p2, cv::Scalar(0, 0, 0), 1);
    }

    // Display the image on a window
    cv::imshow("Grid", image);
    cv::waitKey(0); // Wait for user input to close the window
}

const Eigen::MatrixXi &Grid::data() { return m_grid; }

std::vector<Point> Grid::FindShorestPath(const Point &start, const Point &dest)
{
    std::vector<Point> path = FindShorestPathFunc(start, dest);
    if(SHOW_PATH)
        for(Point p: path) m_grid(p.x, p.y) = m_option.path;
    return path;
}

void Grid::ReadGridFile(const std::string &grid_file)
{
    std::ifstream file(grid_file);
    if (!file.is_open())
    {
        std::cout << "Error: Unable to open file." << std::endl;
        return;
    }

    std::vector<std::vector<int>> grid_data;
    std::string line;
    while (std::getline(file, line))
    {
        std::vector<int> row;
        std::istringstream iss(line);
        int value;
        while (iss >> value)
            row.push_back(value);

        grid_data.push_back(row);
    }

    // Check the size of the grid data
    if (grid_data.empty() || grid_data[0].empty())
    {
        std::cout << "Error: Invalid grid data." << std::endl;
        return;
    }

    // Convert the grid data to an Eigen matrix
    m_grid = Eigen::MatrixXi::Zero(grid_data.size(), grid_data[0].size());
    for (uint i = 0; i < grid_data.size(); i++)
        m_grid.row(i) = Eigen::Map<Eigen::RowVectorXi>(grid_data[i].data(), grid_data[i].size());
}
