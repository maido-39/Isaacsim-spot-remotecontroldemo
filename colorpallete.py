"""
Color Palette Visualization
Displays a well-divided color palette with 10 distinct colors using HSL color space.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import colorsys
import random


def hsl_to_rgb(h, s, l):
    """
    Convert HSL to RGB.
    
    Args:
        h: Hue in degrees (0-360)
        s: Saturation in percent (0-100)
        l: Lightness in percent (0-100)
    
    Returns:
        RGB tuple (0-1 range)
    """
    # Convert to colorsys format (0-1 range)
    h_norm = (h % 360) / 360.0
    s_norm = s / 100.0
    l_norm = l / 100.0
    
    # colorsys uses HLS (same as HSL, just different order)
    return colorsys.hls_to_rgb(h_norm, l_norm, s_norm)


def rgb_to_hsl(r, g, b):
    """
    Convert RGB to HSL.
    
    Args:
        r, g, b: RGB values (0-1 range)
    
    Returns:
        HSL tuple (h in degrees 0-360, s and l in percent 0-100)
    """
    # colorsys uses HLS (same as HSL, just different order)
    h, l, s = colorsys.rgb_to_hls(r, g, b)
    
    # Convert to standard HSL format
    h_deg = (h * 360) % 360
    s_percent = s * 100
    l_percent = l * 100
    
    return (h_deg, s_percent, l_percent)


def generate_well_divided_palette(n=10):
    """
    Generate a well-divided color palette using HSL color space.
    Colors are evenly distributed in hue with 100% saturation and 50% lightness.
    
    Args:
        n: Number of colors to generate (default: 10)
    
    Returns:
        List of RGB tuples (0-1 range) and HSL tuples
    """
    colors_rgb = []
    colors_hsl = []
    
    for i in range(n):
        # Distribute hues evenly (0-360 degrees)
        hue = (i / n) * 360
        
        # Use 100% saturation and 50% lightness for vibrant, balanced colors
        saturation = 100.0
        lightness = 50.0
        
        hsl = (hue, saturation, lightness)
        rgb = hsl_to_rgb(hue, saturation, lightness)
        
        colors_rgb.append(rgb)
        colors_hsl.append(hsl)
    
    return colors_rgb, colors_hsl


def generate_random_palette(n=10):
    """
    Generate a random color palette using HSL with random hue, 100% saturation, 50% lightness.
    
    Args:
        n: Number of colors to generate (default: 10)
    
    Returns:
        List of RGB tuples (0-1 range) and HSL tuples
    """
    colors_rgb = []
    colors_hsl = []
    
    for i in range(n):
        # Random hue (0-360 degrees)
        hue = random.uniform(0, 360)
        
        # Fixed: 100% saturation and 50% lightness
        saturation = 100.0
        lightness = 50.0
        
        hsl = (hue, saturation, lightness)
        rgb = hsl_to_rgb(hue, saturation, lightness)
        
        colors_rgb.append(rgb)
        colors_hsl.append(hsl)
    
    return colors_rgb, colors_hsl


def visualize_palette(colors_rgb, colors_hsl, save_path=None):
    """
    Visualize the color palette in multiple ways.
    
    Args:
        colors_rgb: List of RGB tuples (0-1 range)
        colors_hsl: List of HSL tuples (h in degrees, s and l in percent)
        save_path: Optional path to save the visualization
    """
    fig = plt.figure(figsize=(14, 10))
    
    # 1. Color swatches in a grid
    ax1 = plt.subplot(2, 2, 1)
    n = len(colors_rgb)
    cols = 5  # 5 columns
    rows = (n + cols - 1) // cols
    
    for i, color in enumerate(colors_rgb):
        row = i // cols
        col = i % cols
        rect = patches.Rectangle((col, rows - row - 1), 0.9, 0.9, 
                                facecolor=color, edgecolor='black', linewidth=2)
        ax1.add_patch(rect)
        
        # Add color index
        ax1.text(col + 0.45, rows - row - 0.55, f'{i}', 
                ha='center', va='center', fontsize=12, fontweight='bold',
                color='white' if sum(color) < 1.5 else 'black')
    
    ax1.set_xlim(-0.1, cols)
    ax1.set_ylim(-0.1, rows)
    ax1.set_aspect('equal')
    ax1.axis('off')
    ax1.set_title('Color Palette - Grid View', fontsize=14, fontweight='bold', pad=20)
    
    # 2. Horizontal color bars
    ax2 = plt.subplot(2, 2, 2)
    for i, (color, hsl) in enumerate(zip(colors_rgb, colors_hsl)):
        rect = patches.Rectangle((0, i), 1, 0.8, 
                                facecolor=color, edgecolor='black', linewidth=1.5)
        ax2.add_patch(rect)
        # Add HSL values as text
        h, s, l = hsl
        hsl_str = f"HSL: ({h:.0f}°, {s:.0f}%, {l:.0f}%)"
        ax2.text(1.05, i + 0.4, hsl_str, va='center', fontsize=9,
                color='white' if sum(color) < 1.5 else 'black')
    
    ax2.set_xlim(0, 3.5)
    ax2.set_ylim(-0.1, n)
    ax2.set_aspect('equal')
    ax2.axis('off')
    ax2.set_title('Color Palette - List View (HSL)', fontsize=14, fontweight='bold', pad=20)
    
    # 3. Circular color wheel
    ax3 = plt.subplot(2, 2, 3, projection='polar')
    n = len(colors_rgb)
    theta = np.linspace(0, 2 * np.pi, n, endpoint=False)
    width = 2 * np.pi / n
    
    for i, (t, color) in enumerate(zip(theta, colors_rgb)):
        bars = ax3.bar(t, 1, width=width, color=color, edgecolor='black', linewidth=2)
        # Add label
        label_angle = t + width / 2
        ax3.text(label_angle, 1.2, f'{i}', ha='center', va='center', 
                fontsize=10, fontweight='bold')
    
    ax3.set_ylim(0, 1.5)
    ax3.set_xticks([])
    ax3.set_yticks([])
    ax3.spines['polar'].set_visible(False)
    ax3.set_title('Color Palette - Circular View', fontsize=14, fontweight='bold', pad=20)
    
    # 4. Color spectrum bar
    ax4 = plt.subplot(2, 2, 4)
    width = 256
    height = 50
    gradient = np.zeros((height, width, 3))
    # Create gradient by interpolating between colors
    positions = np.linspace(0, 1, n)
    x_coords = np.linspace(0, 1, width)
    
    for i in range(len(colors_rgb) - 1):
        start_pos = positions[i]
        end_pos = positions[i + 1]
        mask = (x_coords >= start_pos) & (x_coords < end_pos)
        if i == len(colors_rgb) - 2:
            mask = (x_coords >= start_pos) & (x_coords <= end_pos)
        
        # Interpolate between colors
        t = (x_coords[mask] - start_pos) / (end_pos - start_pos)
        interpolated = np.array(colors_rgb[i]) * (1 - t[:, np.newaxis]) + np.array(colors_rgb[i + 1]) * t[:, np.newaxis]
        gradient[:, mask, :] = interpolated[np.newaxis, :, :]
    
    ax4.imshow(gradient, aspect='auto', extent=[0, 1, 0, 1])
    ax4.set_xlim(0, 1)
    ax4.set_ylim(0, 1)
    ax4.axis('off')
    ax4.set_title('Color Palette - Spectrum View', fontsize=14, fontweight='bold', pad=20)
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Visualization saved to {save_path}")
    
    plt.show()


def print_color_info(colors_rgb, colors_hsl):
    """
    Print detailed information about each color.
    
    Args:
        colors_rgb: List of RGB tuples (0-1 range)
        colors_hsl: List of HSL tuples (h in degrees, s and l in percent)
    """
    print("\n" + "="*60)
    print("COLOR PALETTE INFORMATION (HSL)")
    print("="*60)
    for i, (color, hsl) in enumerate(zip(colors_rgb, colors_hsl)):
        r, g, b = [int(c * 255) for c in color]
        h, s, l_val = hsl
        hex_color = f"#{r:02x}{g:02x}{b:02x}"
        print(f"\nColor {i}:")
        print(f"  HSL: ({h:6.1f}°, {s:5.1f}%, {l_val:5.1f}%)")
        print(f"  RGB: ({r:3d}, {g:3d}, {b:3d})")
        print(f"  Hex: {hex_color}")


def get_palette_as_hex(colors):
    """
    Convert RGB colors to hex format.
    
    Args:
        colors: List of RGB tuples (0-1 range)
    
    Returns:
        List of hex color strings
    """
    hex_colors = []
    for color in colors:
        r, g, b = [int(c * 255) for c in color]
        hex_colors.append(f"#{r:02x}{g:02x}{b:02x}")
    return hex_colors


if __name__ == "__main__":
    import sys
    
    # Check if user wants random colors
    use_random = "--random" in sys.argv or "-r" in sys.argv
    
    if use_random:
        print("Generating RANDOM color palette using HSL(random, 100%, 50%)...")
        colors_rgb, colors_hsl = generate_random_palette(10)
        save_path = "color_palette_random_visualization.png"
    else:
        print("Generating WELL-DIVIDED color palette using HSL...")
        colors_rgb, colors_hsl = generate_well_divided_palette(10)
        save_path = "color_palette_visualization.png"
    
    # Print color information
    print_color_info(colors_rgb, colors_hsl)
    
    # Visualize the palette
    visualize_palette(colors_rgb, colors_hsl, save_path=save_path)
    
    # Print hex colors for easy copy-paste
    print("\n" + "="*60)
    print("HEX COLORS (for easy copy-paste):")
    print("="*60)
    hex_colors = get_palette_as_hex(colors_rgb)
    for i, hex_color in enumerate(hex_colors):
        print(f"Color {i}: {hex_color}")
    print("="*60)
    print("\nUsage: python colorpallete.py [--random|-r] for random colors")
    print("="*60 + "\n")

