using Akka.Actor;

namespace Simulation.Dummy
{
    internal class ProductionManager : ReceiveActor
    {
        public ProductionManager()
        {
            Receive<RequestQueueProduction>(req =>
            {
                if (req.Producer.State == Unit.State.Alive && req.Producer.Queue.Count < req.Producer.MaxQueueCount)
                {
                    req.Producer.AddQueue(req.ID);
                    Sender.Tell(new ProductionQueued(true));
                }
                else
                {
                    Sender.Tell(new ProductionQueued(false));
                }
            });
        }
    }
}
