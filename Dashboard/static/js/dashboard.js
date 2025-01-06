// Select gauge and camera feed elements
const gaugeElement = document.querySelector(".gauge");
const cameraFeed = document.getElementById("cameraFeed");

/**
 * Initializes the user's camera feed.
 */
function startCameraFeed() {
    cameraFeed.src = "/video_feed";
}

/**
 * Sets the gauge value, updates rotation of the needle and the text inside the gauge.
 * @param {HTMLElement} gauge - The gauge container element.
 * @param {number} rpm - The RPM value (0 to 8000).
 */
function setGaugeValue(gauge, rpm) {
    const maxRpm = 8000; // Maximum RPM for the gauge
    if (rpm < 0 || rpm > maxRpm) return;

    const value = rpm / maxRpm;
    gauge.querySelector(".gauge__fill").style.transform = `rotate(${value / 2}turn)`;

    const color = getColorFromStops(value);
    gauge.querySelector(".gauge__fill").style.backgroundColor = color;

    gauge.querySelector(".gauge__cover").textContent = `${Math.round(rpm)} RPM`;
}

/**
 * Returns a color from blue → green → red.
 * @param {number} value - Value from 0 to 1 representing the gauge fill percentage.
 */
function getColorFromStops(value) {
    if (value <= 0.5) {
        return interpolateColor("#0000FF", "#00FF00", value * 2);
    } else {
        return interpolateColor("#00FF00", "#FF0000", (value - 0.5) * 2);
    }
}

/**
 * Interpolates between two colors.
 */
function interpolateColor(color1, color2, factor) {
    const r1 = parseInt(color1.slice(1, 3), 16);
    const g1 = parseInt(color1.slice(3, 5), 16);
    const b1 = parseInt(color1.slice(5, 7), 16);
    const r2 = parseInt(color2.slice(1, 3), 16);
    const g2 = parseInt(color2.slice(3, 5), 16);
    const b2 = parseInt(color2.slice(5, 7), 16);
    const r = Math.round(r1 + (r2 - r1) * factor);
    const g = Math.round(g1 + (g2 - g1) * factor);
    const b = Math.round(b1 + (b2 - b1) * factor);
    return `rgb(${r},${g},${b})`;
}

// Simulate RPM changes
let rpm = 0;
setInterval(() => {
    rpm = (rpm + 50) % 8000; // Simulate RPM increase
    setGaugeValue(gaugeElement, rpm);
}, 50);

// Start the camera feed when the page loads
startCameraFeed();