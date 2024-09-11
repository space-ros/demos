#include <string>
#include <gz/msgs.hh>
#include <gz/transport.hh>
#include <gz/sim/System.hh>

#define SCALE_CONST 0.9
#define RATE_CONST 4.0

// used if rain is set as high or low
// random int between these can be used if rain is medium
#define MAX_VEL 5.0      

// rain drop maturity means that as the drop falls it gathers more water and falls faster and becomes more blurry
namespace drone_dust
{
    class DroneDustPrivate;

    class DroneDust:
    public gz::sim::System,
    public gz::sim::ISystemConfigure
{
    public: DroneDust();

    public: ~DroneDust() override;

    public: void Configure(const gz::sim::Entity &_entity,
                    const std::shared_ptr<const sdf::Element> &_sdf,
                    gz::sim::EntityComponentManager &_ecm,
                    gz::sim::EventManager &_eventMgr) override;

    private: std::unique_ptr<DroneDustPrivate> dataPtr;
                
};
}