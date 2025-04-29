#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// ASN.1 Header von asn1c generiert:
extern "C" {
  #include "MCM.h"
  #include "asn_application.h"
  #include "asn_internal.h"
  #include "per_encoder.h"
}

class ConverterNode : public rclcpp::Node {
public:
  ConverterNode() : Node("mcm_converter_node") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "mcm_input",
      10,
      std::bind(&ConverterNode::topic_callback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(this->get_logger(), "MCM Converter Node gestartet");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Empfange ROS2 Nachricht: '%s'", msg->data.c_str());

    // Dummy-Daten erzeugen (hier müsste eigentlich dein Mapping von ROS2 → ASN1-Struktur stehen)
    MCM_t mcm{};
    mcm.header.protocolVersion = 1;
    mcm.header.messageID = 42;
    mcm.header.stationID = 12345;

    // ASN.1 PER-Encoding
    uint8_t buffer[1024];
    asn_enc_rval_t ec = uper_encode_to_buffer(&asn_DEF_MCM, &mcm, buffer, sizeof(buffer));

    if (ec.encoded == -1) {
      RCLCPP_ERROR(this->get_logger(), "Encoding fehlgeschlagen.");
    } else {
      RCLCPP_INFO(this->get_logger(), "MCM erfolgreich in ASN.1 PER kodiert (%ld bit)", ec.encoded);
      // (Optional) Hier könntest du das `buffer` weiterverwenden oder speichern
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConverterNode>());
  rclcpp::shutdown();
  return 0;
}

