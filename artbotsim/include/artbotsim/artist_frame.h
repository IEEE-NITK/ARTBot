#include <QFrame>
#include <QImage>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>
#include <QVector>

// This prevents a MOC error with versions of boost >= 1.48
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <rclcpp/rclcpp.hpp>
# include <ament_index_cpp/get_package_share_directory.hpp>

# include <rcl_interfaces/msg/parameter_event.hpp>
# include <std_srvs/srv/empty.hpp>
# include <artbotsim/srv/spawn.hpp>
# include <artbotsim/srv/kill.hpp>
# include <map>

# include "artist.h"
#endif

namespace artbotsim
{

class ArtistFrame : public QFrame
{
  Q_OBJECT
public:
  ArtistFrame(rclcpp::Node::SharedPtr& node_handle, QWidget* parent = 0, Qt::WindowFlags f = Qt::WindowFlags());
  ~ArtistFrame();

  std::string spawnArtist(const std::string& name, float x, float y, float angle);
  std::string spawnArtist(const std::string& name, float x, float y, float angle, size_t index);

protected:
  void paintEvent(QPaintEvent* event);

private slots:
  void onUpdate();

private:
  void updateArtists();
  void clear();
  bool hasArtist(const std::string& name);

  bool clearCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);
  bool resetCallback(const std_srvs::srv::Empty::Request::SharedPtr, std_srvs::srv::Empty::Response::SharedPtr);
  bool spawnCallback(const artbotsim::srv::Spawn::Request::SharedPtr, artbotsim::srv::Spawn::Response::SharedPtr);
  bool killCallback(const artbotsim::srv::Kill::Request::SharedPtr, artbotsim::srv::Kill::Response::SharedPtr);

  void parameterEventCallback(const rcl_interfaces::msg::ParameterEvent::ConstSharedPtr);

  rclcpp::Node::SharedPtr nh_;

  QTimer* update_timer_;
  QImage path_image_;
  QPainter path_painter_;

  uint64_t frame_count_;

  rclcpp::Time last_artist_update_;

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  rclcpp::Service<artbotsim::srv::Spawn>::SharedPtr spawn_srv_;
  rclcpp::Service<artbotsim::srv::Kill>::SharedPtr kill_srv_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  typedef std::map<std::string, ArtistPtr> M_Artist;
  M_Artist artists_;
  uint32_t id_counter_;

  QVector<QImage> artist_images_;

  float meter_;
  float width_in_meters_;
  float height_in_meters_;
};

}