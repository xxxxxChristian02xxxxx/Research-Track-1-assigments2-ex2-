// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <inttypes.h>
#include <memory>
#include "service_msgs/srv/stop_start.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

using StopStart = service_msgs::srv::StopStart;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class StopStartServer : public rclcpp::Node
{
public:
  StopStartServer()
  : Node("stop_start_server")
  {
    service_ = this->create_service<service_msgs::srv::StopStart>("/stop_start", std::bind(&StopStartServer::handle_service, this, _1, _2, _3));
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
    // qui sono sempre lato server, in sto modo, sto usando il server che si trova in un nodo a se stante,
    // il quale semplicemente vede ciò che ha nella request e lo raddoppia per la response quando viene chiamato

void handle_service(  const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<service_msgs::srv::StopStart::Request> request,
        std::shared_ptr<service_msgs::srv::StopStart::Response> response)
    {
        
        geometry_msgs::msg::Twist vel;
        if (request->stop_start)
        {
            vel.linear.x = 0.0;
            vel.linear.y = 0.0;
            vel.angular.z = 0.0;      
        }
        
        publisher_->publish(vel);
        response->success = true;
    }

    rclcpp::Service<service_msgs::srv::StopStart>::SharedPtr service_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StopStartServer>());
  rclcpp::shutdown();
  return 0;
}