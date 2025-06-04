using Akka.Actor;

namespace Simulation.Dummy
{
    internal class TransportManager : ReceiveActor
    {
        public TransportManager()
        {
            Receive<RequestTransportAllocation>(req =>
            {
                if (req.Mover.State == Unit.State.Alive && req.Mover.ServiceRequester == ActorRefs.Nobody)
                {
                    req.Mover.Allocate(req.Product);
                    Sender.Tell(new TransportAllocated(true));
                }
                else
                {
                    Sender.Tell(new TransportAllocated(false));
                }
            });
        }
    }
}
