using Simulation.Util;
using Vector2 = System.Numerics.Vector2;
using Vector4 = System.Numerics.Vector4;

namespace Simulation.UserInterface
{
    /// <summary>
    /// A UI slider component that allows the user to dynamically adjust an interval.
    /// </summary>
    /// <param name="id">Unique ID used to identify rendering draw command.</param>
    /// <param name="position">The position of the component.</param>
    /// <param name="slider"> X ball radius, Y bar width.</param>
    /// <param name="barColor">The color of the slider bar.</param>
    /// <param name="ballColor">The color of the slider ball.</param>
    /// <param name="domainBreak">
    ///   Normalized fraction in [0,1] at which the slider’s easing curve “kinks” from the first linear
    ///   segment into the second. Inputs below this use the first slope; above use the second.
    /// </param>
    /// <param name="rangeBreak">
    ///   Normalized ratio in [0,1] at which the output of the first segment ends and the second begins.
    ///   This is the output value corresponding to <paramref name="domainBreak"/>.
    /// </param>/// 
    /// <param name="interval">The initial interval value.</param>
    /// <param name="minInterval">The minimal allowed interval value.</param>
    /// <param name="maxInterval">The maximum allowed interval value.</param>
    internal class Slider(
        string id,
        Vector2 position,
        Vector2 slider,
        Vector4 barColor,
        Vector4 ballColor,
        uint interval = 0,
        uint minInterval = 0,
        uint maxInterval = 1,
        bool visible = false,
        float domainBreak = 1,
        float rangeBreak = 1
    ) : UIEventComponent($"{id}_slider{position.X}-{position.Y}")
    {
        private readonly Vector2 _position = position;
        private readonly float _ballRadius = slider.X;
        private readonly float _barWidth = slider.Y;
        private readonly uint _originalInterval = interval;
        private readonly uint _minInterval = minInterval;
        private readonly uint _maxInterval = maxInterval;

        /// <summary>
        /// Strategy object performing the piecewise linear conversion between:
        /// <list type="bullet">
        ///   <item><description><see cref="PiecewiseLinearMap.DomainBreak"/>: the normalized interval fraction at which the slope changes.</description></item>
        ///   <item><description><see cref="PiecewiseLinearMap.RangeBreak"/>: the normalized slider ratio at which the slope changes.</description></item>
        /// </list>
        /// <para/>
        /// Use <see cref="_mapper.MapDomainToRange(frac)"/> to turn an interval‐fraction into a slider‐ratio,
        /// and <see cref="_mapper.MapRangeToDomain(ratio)"/> to go back from ratio to fraction.
        /// </summary>
        private readonly PiecewiseLinearMap _mapper = new(domainBreak, rangeBreak);

        private Vector2 _renderPos;
        private float _renderBallRadius;
        private float _renderBarWidth;

        private float _stepSize;
        private float _ballPos;
        private uint _interval = interval;

        internal Vector4 BarColor { get; set; } = barColor;
        internal Vector4 BallColor { get; set; } = ballColor;

        /// <summary>Gets or sets the current interval and updates the slider ball position.</summary>
        internal uint Interval
        {
            get => _interval;
            set
            {
                if (_interval != value)
                {
                    _interval = value;
                    UpdateBall();
                }
            }
        }

        internal bool Visible { get; set; } = visible;
        internal bool Active
        {
            get
            {
                return _maxInterval - _minInterval == 1 && _interval == _maxInterval;
            }
        }

        internal bool Clicked { get; set; } = false;
        internal bool Hovered { get; set; } = false;

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
            Hovered = true;
            UpdateInterval();
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
            if (!Hovered)
                return;

            if (X < _renderPos.X - _renderBallRadius)
            {
                _ballPos = _renderPos.X;
                Hovered = false;
            }
            else if (X > _renderPos.X + _renderBarWidth + _renderBallRadius)
            {
                _ballPos = _renderPos.X + _renderBarWidth;
                Hovered = false;
            }
            else
            {
                _ballPos = Math.Clamp(
                    X,
                    _renderPos.X,
                    _renderPos.X + _renderBarWidth
                    );
            }
            OnHover();
        }

        internal override void OnHover()
        {
            UpdateInterval();
            Render();
        }

        /// <summary>Ends the dragging interaction when the mouse is released.</summary>
        internal override bool LeftRelease()
        {
            if (Clicked)
            {
                OnLeftRelease();
                Clicked = false;
                Hovered = false;
                return true;
            }
            Clicked = false;
            return false;
        }

        internal override void OnLeftRelease()
        {
            UpdateInterval();
            Render();
        }

        internal override void UpdateViewport(float scale)
        {
            _renderPos = _position * scale;
            _renderBallRadius = _ballRadius * scale;
            _renderBarWidth = _barWidth * scale;

            _stepSize = _renderBarWidth / (_maxInterval - _minInterval);

            _ballPos =
                _renderPos.X +
                _renderBarWidth -
                _stepSize * (_maxInterval - _interval)
                ;

            UpdateBall();
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

        /// <summary>Recalculates the interval based on the position of the current slider ball position.</summary>
        private void UpdateInterval()
        {
            if (_maxInterval - _minInterval == 1)
            {
                _interval ^= 0b1;
                return;
            }

            var ratio = (_ballPos - _renderPos.X) / _renderBarWidth;
            var mappedFrac = _mapper.MapRangeToDomain(ratio);

            _interval = (uint)(_minInterval + (_maxInterval - _minInterval) * mappedFrac);
        }

        /// <summary>Recalculates the ball position based on the current interval.</summary>
        private void UpdateBall()
        {
            if (_maxInterval - _minInterval == 1)
            {
                _ballPos = _renderPos.X + _interval * _stepSize;
                return;
            }

            var frac = (_interval - _minInterval) / (float)(_maxInterval - _minInterval);
            var sliderRatio = _mapper.MapDomainToRange(frac);

            _ballPos = _renderPos.X + sliderRatio * _renderBarWidth;
        }

        internal override void Remove()
        {
            Renderer.Instance.RemoveDrawCommand(ID);
        }

        internal override void Reset()
        {
            if (_interval != _originalInterval)
            {
                _interval = _originalInterval;
                UpdateBall();
                UpdateInterval();
                Render();
            }
        }
    }
}
