#include "artbotsim/artist_frame.h"

#include <QPointF>

#include <cstdlib>
#include <ctime>

#define DEFAULT_BG_R 0x5E
#define DEFAULT_BG_G 0x5E
#define DEFAULT_BG_B 0x5E

namespace artbotsim
{

ArtistFrame::ArtistFrame(rclcpp::Node::SharedPtr& node_handle, QWidget* parent, Qt::WindowFlags f)
: QFrame(parent, f)
, path_image_(800, 640, QImage::Format_ARGB32)
, path_painter_(&path_image_)
, frame_count_(0)
, id_counter_(0)
{
  setFixedSize(800, 640);
  setWindowTitle("artbotsim");

  srand(time(NULL));

  update_timer_ = new QTimer(this);
  update_timer_->setInterval(16);
  update_timer_->start();

  connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

  nh_ = node_handle;
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = 0;
  range.step = 1;
  range.to_value = 255;
  rcl_interfaces::msg::ParameterDescriptor background_r_descriptor;
  background_r_descriptor.description = "Red channel of the background color";
  background_r_descriptor.integer_range.push_back(range);
  rcl_interfaces::msg::ParameterDescriptor background_g_descriptor;
  background_g_descriptor.description = "Green channel of the background color";
  background_g_descriptor.integer_range.push_back(range);
  rcl_interfaces::msg::ParameterDescriptor background_b_descriptor;
  background_b_descriptor.description = "Blue channel of the background color";
  background_b_descriptor.integer_range.push_back(range);
  nh_->declare_parameter("background_r", rclcpp::ParameterValue(DEFAULT_BG_R), background_r_descriptor);
  nh_->declare_parameter("background_g", rclcpp::ParameterValue(DEFAULT_BG_G), background_g_descriptor);
  nh_->declare_parameter("background_b", rclcpp::ParameterValue(DEFAULT_BG_B), background_b_descriptor);

  rcl_interfaces::msg::ParameterDescriptor holonomic_descriptor;
  holonomic_descriptor.description = "If true, then artists will be holonomic";
  nh_->declare_parameter("holonomic", rclcpp::ParameterValue(true), holonomic_descriptor);

  QVector<QString> artists;
  artists.append("artist.png");

  QString images_path = (ament_index_cpp::get_package_share_directory("artbotsim") + "/images/").c_str();
  for (int i = 0; i < artists.size(); ++i)
  {
    QImage img;
    img.load(images_path + artists[i]);
    artist_images_.append(img);
  }

  meter_ = artist_images_[0].height();

  clear();

  clear_srv_ = nh_->create_service<std_srvs::srv::Empty>("clear", std::bind(&ArtistFrame::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = nh_->create_service<std_srvs::srv::Empty>("reset", std::bind(&ArtistFrame::resetCallback, this, std::placeholders::_1, std::placeholders::_2));
  spawn_srv_ = nh_->create_service<artbotsim::srv::Spawn>("spawn", std::bind(&ArtistFrame::spawnCallback, this, std::placeholders::_1, std::placeholders::_2));
  kill_srv_ = nh_->create_service<artbotsim::srv::Kill>("kill", std::bind(&ArtistFrame::killCallback, this, std::placeholders::_1, std::placeholders::_2));

  rclcpp::QoS qos(rclcpp::KeepLast(100), rmw_qos_profile_sensor_data);
  parameter_event_sub_ = nh_->create_subscription<rcl_interfaces::msg::ParameterEvent>(
    "/parameter_events", qos, std::bind(&ArtistFrame::parameterEventCallback, this, std::placeholders::_1));

  RCLCPP_INFO(nh_->get_logger(), "Starting artbotsim with node name %s", nh_->get_fully_qualified_name());

  width_in_meters_ = (width() - 1) / meter_;
  height_in_meters_ = (height() - 1) / meter_;
  spawnArtist("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);

  // spawn all available artist types
  if(false)
  {
    for(int index = 0; index < artists.size(); ++index)
    {
      QString name = artists[index];
      name = name.split(".").first();
      name.replace(QString("-"), QString(""));
      spawnArtist(name.toStdString(), 1.0f + 1.5f * (index % 7), 1.0f + 1.5f * (index / 7), static_cast<float>(PI) / 2.0f, index);
    }
  }
}

ArtistFrame::~ArtistFrame()
{
  delete update_timer_;
}

bool ArtistFrame::spawnCallback(const artbotsim::srv::Spawn::Request::SharedPtr req, artbotsim::srv::Spawn::Response::SharedPtr res)
{
  std::string name = spawnArtist(req->name, req->x, req->y, req->theta);
  if (name.empty())
  {
    RCLCPP_ERROR(nh_->get_logger(), "An artist named [%s] already exists", req->name.c_str());
    return false;
  }

  res->name = name;

  return true;
}

bool ArtistFrame::killCallback(const artbotsim::srv::Kill::Request::SharedPtr req, artbotsim::srv::Kill::Response::SharedPtr)
{
  M_Artist::iterator it = artists_.find(req->name);
  if (it == artists_.end())
  {
    RCLCPP_ERROR(nh_->get_logger(), "Tried to kill artist [%s], which does not exist", req->name.c_str());
    return false;
  }

  artists_.erase(it);
  update();

  return true;
}

void ArtistFrame::parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr event)
{
  // only consider events from this node
  if (event->node == nh_->get_fully_qualified_name())
  {
    // since parameter events for this event aren't expected frequently just always call update()
    update();
  }
}

bool ArtistFrame::hasArtist(const std::string& name)
{
  return artists_.find(name) != artists_.end();
}

std::string ArtistFrame::spawnArtist(const std::string& name, float x, float y, float angle)
{
  return spawnArtist(name, x, y, angle, rand() % artist_images_.size());
}

std::string ArtistFrame::spawnArtist(const std::string& name, float x, float y, float angle, size_t index)
{
  std::string real_name = name;
  if (real_name.empty())
  {
    do
    {
      std::stringstream ss;
      ss << "artist" << ++id_counter_;
      real_name = ss.str();
    } while (hasArtist(real_name));
  }
  else
  {
    if (hasArtist(real_name))
    {
      return "";
    }
  }

  ArtistPtr t = std::make_shared<Artist>(nh_, real_name, artist_images_[static_cast<int>(index)], QPointF(x, height_in_meters_ - y), angle);
  artists_[real_name] = t;
  update();

  RCLCPP_INFO(nh_->get_logger(), "Spawning artist [%s] at x=[%f], y=[%f], theta=[%f]", real_name.c_str(), x, y, angle);

  return real_name;
}

void ArtistFrame::clear()
{
  // make all pixels fully transparent
  path_image_.fill(qRgba(255, 255, 255, 0));
  update();
}

void ArtistFrame::onUpdate()
{
  if (!rclcpp::ok())
  {
    close();
    return;
  }

  rclcpp::spin_some(nh_);

  updateArtists();
}

void ArtistFrame::paintEvent(QPaintEvent*)
{
  QPainter painter(this);

  int r = DEFAULT_BG_R;
  int g = DEFAULT_BG_G;
  int b = DEFAULT_BG_B;
  nh_->get_parameter("background_r", r);
  nh_->get_parameter("background_g", g);
  nh_->get_parameter("background_b", b);
  QRgb background_color = qRgb(r, g, b);
  painter.fillRect(0, 0, width(), height(), background_color);

  painter.drawImage(QPoint(0, 0), path_image_);

  M_Artist::iterator it = artists_.begin();
  M_Artist::iterator end = artists_.end();
  for (; it != end; ++it)
  {
    it->second->paint(painter);
  }
}

void ArtistFrame::updateArtists()
{
  if (last_artist_update_.nanoseconds() == 0)
  {
    last_artist_update_ = nh_->now();
    return;
  }

  bool modified = false;
  M_Artist::iterator it = artists_.begin();
  M_Artist::iterator end = artists_.end();
  for (; it != end; ++it)
  {
    modified |= it->second->update(0.001 * update_timer_->interval(), path_painter_, path_image_, width_in_meters_, height_in_meters_);
  }
  if (modified)
  {
    update();
  }

  ++frame_count_;
}


bool ArtistFrame::clearCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Clearing artbotsim.");
  clear();
  return true;
}

bool ArtistFrame::resetCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(nh_->get_logger(), "Resetting artbotsim.");
  artists_.clear();
  id_counter_ = 0;
  spawnArtist("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);
  clear();
  return true;
}

}