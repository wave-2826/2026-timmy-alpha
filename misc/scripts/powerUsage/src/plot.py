import matplotlib
import matplotlib.pyplot as plt
from .analyze import LogResult
import bisect

def interactive_legend(leg: plt.Legend):
    map_legend_to_ax = {}  # Will map legend lines to original lines.

    pickradius = 5  # Points (Pt). How close the click needs to be to trigger an event.

    fig = leg.figure
    lines = leg.axes.get_lines()
    
    # Discluding ones not in the legend
    lines = [line for line in lines if any(line.get_label() == leg_line.get_label() for leg_line in leg.get_lines())]

    for legend_line, ax_line in zip(leg.get_lines(), lines):
        legend_line.set_picker(pickradius) # Enable picking on the legend line.
        map_legend_to_ax[legend_line] = ax_line

    def on_pick(event):
        # On the pick event, find the original line corresponding to the legend
        # proxy line, and toggle its visibility.
        legend_line = event.artist

        # Do nothing if the source of the event is not a legend line.
        if legend_line not in map_legend_to_ax:
            return

        ax_line = map_legend_to_ax[legend_line]
        visible = not ax_line.get_visible()
        ax_line.set_visible(visible)
        # Change the alpha on the line in the legend, so we can see what lines
        # have been toggled.
        legend_line.set_alpha(1.0 if visible else 0.2)
        fig.canvas.draw()

    fig.canvas.mpl_connect('pick_event', on_pick)

    # Works even if the legend is draggable. This is independent from picking legend lines.
    leg.set_draggable(True)

def add_tooltips_to_bars(ax, bars, labels):
    tooltip = ax.annotate("", xy=(0,0), xytext=(10,10), textcoords="offset points",
        bbox=dict(boxstyle="round", fc="w"), zorder=100)
    tooltip.set_visible(False)

    def on_motion(event):
        visible = tooltip.get_visible()
        if event.inaxes == ax:
            for bar, label in zip(bars, labels):
                contains, _ = bar.contains(event)
                if contains:
                    tooltip.xy = (event.xdata, event.ydata)
                    tooltip.set_text(label)
                    tooltip.set_visible(True)
                    event.canvas.draw_idle()
                    return
        if visible:
            tooltip.set_visible(False)
            event.canvas.draw_idle()

    ax.figure.canvas.mpl_connect("motion_notify_event", on_motion)

def add_value_tooltip_to_lines(ax, lines):
    tooltip = ax.annotate("", xy=(0,0), xytext=(10,10), textcoords="offset points",
        bbox=dict(boxstyle="round", fc="w"), zorder=100)
    tooltip.set_visible(False)

    def on_motion(event):
        visible = tooltip.get_visible()
        if event.inaxes == ax:
            closest_line = None
            min_dist = float('inf')
            
            for line in lines:
                if not line.get_visible():
                    continue
                if not line.contains(event)[0]:
                    continue

                xdata = line.get_xdata()
                ydata = line.get_ydata()
                if len(xdata) == 0:
                    continue
                
                # Binary search for the closest x coordinate
                idx = bisect.bisect_left(xdata, event.xdata)
                
                dist = float('inf')
                for i in (idx - 1, idx):
                    if 0 <= i < len(xdata):
                        display_coords = ax.transData.transform((xdata[i], ydata[i]))
                        mouse_display_coords = (event.x, event.y)
                        d = ((display_coords[0] - mouse_display_coords[0]) ** 2 + 
                             (display_coords[1] - mouse_display_coords[1]) ** 2) ** 0.5
                        dist = min(dist, d)
                        
                if dist < min_dist:
                    min_dist = dist
                    closest_line = line
                    
            if closest_line is not None and min_dist < 100:
                tooltip.xy = (event.xdata, event.ydata)
                tooltip.set_text(closest_line.get_label())
                tooltip.set_visible(True)
                tooltip.get_bbox_patch().set_edgecolor(closest_line.get_color())
                event.canvas.draw_idle()
                return
        
        if visible:
            tooltip.set_visible(False)
            event.canvas.draw_idle()

    ax.figure.canvas.mpl_connect("motion_notify_event", on_motion)

def plot_integrals(ty, units, log_results: list[LogResult]):
    # Power integral plot
    plt.figure(figsize=(12, 6))

    # Draw a light red box in the background for auto and a light green box for teleop
    max_energy = max(result.integral(ty).last_or(0) for result in log_results) * 1.1
    plt.gca().add_patch(plt.Rectangle((0, 0), 20, max_energy, facecolor="red", alpha=0.1))
    plt.gca().add_patch(plt.Rectangle((23, 0), 140, max_energy, facecolor="green", alpha=0.1))

    lines = []
    for result in log_results:
        timestamps = result.integral(ty).timestamps
        energies = result.integral(ty).values
        lines.extend(
            plt.plot([ts - result.start_offset for ts in timestamps], energies, label=f"{result.name} (Total: {energies[-1]:.2f} {units}, {len(result.brownout_timestamps)} brownouts)")
        )
        # Plot brownouts as red dots
        for brownout_ts in result.brownout_timestamps:
            plt.plot(brownout_ts - result.start_offset, [
                result.integral(ty).get_nearest(brownout_ts) or 0
            ], 'ro', "", markersize=2)
    add_value_tooltip_to_lines(plt.gca(), lines)

    interactive_legend(plt.legend(fancybox=True, shadow=True))
    plt.grid(axis="both", linestyle="--", alpha=0.7)

def plot(log_results: list[LogResult]):
    print("Plotting results...")

    # Reduce margin around edges for all figures
    plt.rcParams['figure.subplot.left'] = 0.05
    plt.rcParams['figure.subplot.right'] = 0.95
    plt.rcParams['figure.subplot.top'] = 0.95
    plt.rcParams['figure.subplot.bottom'] = 0.05

    # Channel plot
    plt.figure(figsize=(12, 6))
    bar_width = 0.8 / len(log_results)
    for log_idx, result in enumerate(log_results):
        total_power = sum(result.power_sum_per_channel)
        plt.bar([x + log_idx * bar_width for x in range(24)], result.power_sum_per_channel, width=bar_width, label=f"{result.name} (Total: {total_power:.2f} Wh, {len(result.brownout_timestamps)} brownouts)")
    plt.xlabel("Channel")
    plt.ylabel("Total Energy (Wh)")
    plt.title("Total Energy per Channel")
    plt.legend()
    plt.xticks(range(24))
    plt.grid(axis="y", linestyle="--", alpha=0.7)

    # Per-log plot
    plt.figure(figsize=(12, 6))
    max_brownouts = max(len(result.brownout_timestamps) for result in log_results) + 1
    ax1 = plt.gca()
    ax2 = ax1.twinx()
    x = range(len(log_results))
    width = 0.4
    
    for log_idx, result in enumerate(log_results):
        total_power = sum(result.power_sum_per_channel)
        ax1.bar(
            x[log_idx] - width/2, total_power, width=width,
            label=f"{result.name} (Total: {total_power:.2f} Wh, {len(result.brownout_timestamps)} brownouts)",
            color=plt.cm.RdYlGn_r(min(len(result.brownout_timestamps) / max_brownouts, 1.0))
        )
        ax2.bar(
            x[log_idx] + width/2, result.average_current_while_enabled, width=width*0.5,
            color="#ffff55"
        )

    plt.xlabel("Log")
    plt.title("Total Energy and Average Current per Log")
    plt.legend()
    ax1.set_xticks(x)
    ax1.set_xticklabels([result.name for result in log_results])
    ax1.set_ylabel("Total Energy (Wh)")
    ax2.set_ylabel("Average Current While Enabled (A)")

    plot_integrals("power", "Wh", log_results)
    plt.xlabel("Time (s)")
    plt.ylabel("Cumulative Energy (Wh)")
    plt.yscale("linear")
    plt.title("Cumulative Energy over Time")

    # Power plot
    # plt.figure(figsize=(12, 6))

    # lines = []
    # for result in log_results:
    #     timestamps = result.integral(ty).timestamps
    #     energies = result.integral(ty).values
    #     lines.extend(
    #         plt.plot([ts - result.start_offset for ts in timestamps], energies, label=f"{result.name}")
    #     )
    #     # Plot brownouts as red dots
    #     for brownout_ts in result.brownout_timestamps:
    #         plt.plot(brownout_ts - result.start_offset, [
    #             result.integral(ty).get_nearest(brownout_ts) or 0
    #         ], 'ro', "", markersize=2)
    # add_value_tooltip_to_lines(plt.gca(), lines)

    # interactive_legend(plt.legend(fancybox=True, shadow=True))
    # plt.grid(axis="both", linestyle="--", alpha=0.7)
    # plt.xlabel("Time (s)")
    # plt.ylabel("Power (W)")
    # plt.yscale("linear")
    # plt.title("Power over Time")
    
    # plot_integrals("amperage", "As")
    # plt.xlabel("Time (s)")
    # plt.ylabel("Cumulative Amperage (As)")
    # plt.yscale("linear")
    # plt.title("Cumulative Amperage over Time")

    # Plot logs on the vertical axis and subsystem percentages horizontally with a stacked bar chart to compare subsystems between matches
    subsystem_names = set()
    for result in log_results:
        subsystem_names.update(result.subsystem_results.keys())
    # subsystem_names = sorted(subsystem_names)
    # Sort by rough total power usage across all matches
    subsystem_names: list[str] = sorted(subsystem_names, key=lambda name:
        sum(result.subsystem_results[name].power_sum if name in result.subsystem_results else 0 for result in log_results),
        reverse=True
    )

    plt.figure(figsize=(16, 6))
    
    plt.subplot(1, 2, 1)
    left = [0 for _ in log_results]
    for name in subsystem_names:
        percentages = [result.subsystem_results[name].percentage_total_power if name in result.subsystem_results else 0 for result in log_results]
        bars = plt.barh(
            [result.name for result in log_results], percentages, left=left, label=name
        )
        add_tooltips_to_bars(plt.gca(), bars, [(
            "\n".join([f"""${{{"\\bf" if ss == name else ""} {ss}}}\\text{{: {
                log_results[i].subsystem_results[ss].percentage_total_power * 100. if ss in log_results[i].subsystem_results else 0
            :.2f}%}}$""" for ss in subsystem_names])
        ) for i in range(len(log_results))])
        left = [left[i] + percentages[i] for i in range(len(log_results))]
    
    plt.subplots_adjust(left=0.1, bottom=0.1)
    plt.xlabel("Percentage Power")
    plt.title("Percentage Total Power by Subsystem")

    plt.subplot(1, 2, 2)
    # Remove spacing between left and right plots
    plt.subplots_adjust(wspace=0.05)

    left = [0 for _ in log_results]
    for name in subsystem_names:
        percentages = [result.subsystem_results[name].power_sum if name in result.subsystem_results else 0 for result in log_results]
        bars = plt.barh(
            [result.name for result in log_results], percentages, left=left, label=name
        )
        add_tooltips_to_bars(plt.gca(), bars, [(
            "\n".join([f"""${{{"\\bf" if ss == name else ""} {ss}}}\\text{{: {
                log_results[i].subsystem_results[ss].power_sum if ss in log_results[i].subsystem_results else 0
            :.2f} Wh}}$""" for ss in subsystem_names])
        ) for i in range(len(log_results))])
        left = [left[i] + percentages[i] for i in range(len(log_results))]

    plt.yticks([]) # Hide y-axis labels since they match the left plot
    plt.xlabel("Total Power (Wh)")
    plt.title("Total Power by Subsystem")
    plt.legend(loc="upper right", fancybox=True, ncol=10)

    plt.figure(figsize=(10, 8))
    for log_idx, result in enumerate(log_results):
        plt.plot(result.total_power.timestamps, result.total_power.values, label=result.name)
    plt.legend()
    
    plt.show()