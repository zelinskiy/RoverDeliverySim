using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Input;

namespace RoverDeliverySim
{
    public class GroundClicker : SyncScript
    {
        public CameraComponent Camera;
        public Entity MouseClickSphere { get; set; }
        public Entity Rover { get; set; }
        private float showMouseClickDelay;

        public override void Update()
        {
            if (Input.HasMouse && Input.IsMouseButtonPressed(MouseButton.Left))
            {
                Vector2 mousePos = Input.MousePosition;
                Ray ray = ScreenToWorldRay(mousePos, Camera);
                Plane groundPlane = new Plane(Vector3.Zero, Vector3.UnitY);

                if (ray.Intersects(ref groundPlane, out Vector3 hitPoint))
                {
                    // 1. Show click indicator sphere
                    if (MouseClickSphere != null)
                    {
                        var modelComp = MouseClickSphere.Get<ModelComponent>();
                        if (modelComp != null) modelComp.Enabled = true;

                        showMouseClickDelay = 5.0f;
                        var mouseClickY = MouseClickSphere.Transform.Position.Y;
                        MouseClickSphere.Transform.Position = hitPoint;
                        MouseClickSphere.Transform.Position.Y = mouseClickY;
                    }

                    // 2. Send target position to the Rover!
                    if (Rover != null)
                    {
                        var roverController = Rover.Get<RoverController>();
                        if (roverController != null)
                        {
                            roverController.TargetPosition = hitPoint;
                        }
                    }
                }
            }

            if (showMouseClickDelay > 0)
            {
                showMouseClickDelay -= (float)Game.UpdateTime.Elapsed.TotalSeconds;
                if (showMouseClickDelay <= 0 && MouseClickSphere != null)
                {
                    var modelComp = MouseClickSphere.Get<ModelComponent>();
                    if (modelComp != null) modelComp.Enabled = false;
                }
            }
        }

        private Ray ScreenToWorldRay(Vector2 screenPos, CameraComponent camera)
        {
            Matrix invViewProj = Matrix.Invert(camera.ViewProjectionMatrix);

            Vector3 sPos;
            sPos.X = screenPos.X * 2f - 1f;
            sPos.Y = 1f - screenPos.Y * 2f;

            // Near plane point (Z = 0)
            sPos.Z = 0f;
            Vector4 vectorNear = Vector3.Transform(sPos, invViewProj);
            vectorNear /= vectorNear.W;

            // Far plane point (Z = 1)
            sPos.Z = 1f;
            Vector4 vectorFar = Vector3.Transform(sPos, invViewProj);
            vectorFar /= vectorFar.W;

            Vector3 rayDirection = Vector3.Normalize(vectorFar.XYZ() - vectorNear.XYZ());
            return new Ray(vectorNear.XYZ(), rayDirection);
        }
    }
}