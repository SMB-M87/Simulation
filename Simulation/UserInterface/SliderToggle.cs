using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface
{
    /// <summary>
    /// A UI slider component that allows the user to dynamically adjust an interval.
    /// </summary>
    /// <param name="id">Unique ID used to identify rendering draw command.</param>
    /// <param name="position">The position of the component.</param>
    /// <param name="slider">X: radius of the slider ball, Y width of the bar.</param>
    /// <param name="barColor">The color of the slider bar.</param>
    /// <param name="ballColor">The color of the slider ball.</param>
    /// <param name="interval">The initial interval value.</param>
    internal class SliderToggle(
        string id,
        Vector2 position,
        Vector2 slider,
        Vector4 barColor,
        Vector4 ballColor,
        bool visible = false,
        bool active = false
    ) : UIEventComponent($"{id}_slider_toggle_{position.X}-{position.Y}")
    {
        private readonly Vector2 _position = position;
        private readonly float _ballRadius = slider.X;
        private readonly float _barWidth = slider.Y;
        private readonly bool _originalState = active;
        private readonly bool _originalVisibility = visible;

        private Vector2 _renderPos = position;
        private float _renderBallRadius = slider.X;
        private float _renderBarWidth = slider.Y;

        private float _stepSize = slider.Y;
        private float _ballPos = position.X;
        private bool _active = active;

        internal Vector4 BarColor { get; set; } = barColor;
        internal Vector4 BallColor { get; set; } = ballColor;

        internal bool Visible { get; set; } = visible;
        internal bool Active
        {
            get
            {
                return _active;
            }
            set
            {
                var prev = _active;
                _active = value;

                if (prev != _active)
                {
                    BallPositioning();

                    if (UI.Instance.SettingPanel.Visible)
                        Render();
                }
            }
        }

        internal bool Clicked { get; set; } = false;

        /// <summary>
        /// Handles mouse click interaction for the slider component.
        /// Begins dragging if the ball is clicked, or jumps the slider position if the bar is clicked.
        /// </summary>
        /// <param name="mouseX">The X position of the mouse click.</param>
        /// <param name="mouseY">The Y position of the mouse click.</param>
        internal override bool LeftClick(float X, float Y)
        {
            var radius = _renderBallRadius * 2;

            var left = _renderPos.X;
            var right = left + _renderBarWidth;
            var top = _renderPos.Y - radius / 2;
            var bottom = _renderPos.Y + radius / 2;

            var clickedBall =
                Math.Abs(X - _ballPos) < radius &&
                Math.Abs(Y - _renderPos.Y) < radius;

            var clickedSlider =
                X >= left &&
                X <= right &&
                Y >= top &&
                Y <= bottom;

            if (clickedBall)
            {
                OnLeftClick();
                return true;
            }
            else if (clickedSlider)
            {
                _ballPos = Math.Clamp(
                    X,
                    _renderPos.X,
                    _renderPos.X + _renderBarWidth
                    );

                OnLeftClick();
                return true;
            }
            return false;
        }

        internal override void OnLeftClick()
        {
            Clicked = true;
            _active = !_active;
            BallPositioning();
            Render();
        }

        /// <summary>
        /// Updates the position in real time while the user is dragging the ball.
        /// Adjusts the interval based on the new ball position.
        /// </summary>
        /// <param name="X">The current horizontal mouse position.</param>
        /// <param name="_">Unused Y coordinate.</param>
        internal override void Hover(float X, float _)
        {
            if (!Clicked)
                return;

            OnHover();
        }

        internal override void OnHover()
        {
            Render();
        }

        /// <summary>Ends the dragging interaction when the mouse is released.</summary>
        internal override bool LeftRelease()
        {
            if (Clicked)
            {
                OnLeftRelease();
                return true;
            }
            return false;
        }

        internal override void OnLeftRelease()
        {
            Clicked = false;
            Render();
        }

        internal override void UpdateViewport(float scale)
        {
            _renderPos = _position * scale;
            _renderBallRadius = _ballRadius * scale;
            _renderBarWidth = _barWidth * scale;

            _stepSize = _renderBarWidth;

            _ballPos = _renderPos.X + _stepSize * (_active ? 1.0f : 0.0f);
        }

        internal override void Render()
        {
            Renderer.Instance.DrawSlider(
                ID,
                _renderPos,

                new(_renderBallRadius,
                    _renderBallRadius,
                    _ballPos,
                    _renderBarWidth),

                BarColor,
                BallColor);
        }

        private void BallPositioning()
        {
            _ballPos = _renderPos.X + _stepSize * (_active ? 1.0f : 0.0f);
        }

        internal override void Remove()
        {
            Renderer.Instance.RemoveDrawCommand(ID);
        }

        internal override void Reset()
        {
            _active = _originalState;
            Visible = _originalVisibility;

            BallPositioning();
            Render();
        }
    }
}
