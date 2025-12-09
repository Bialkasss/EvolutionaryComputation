library(tidyverse)
library(ggplot2)
library(showtext)

font_add_google("Roboto Condensed", "roboto")
showtext_auto()

theme_set(
  theme_minimal(base_family = "Arial") +
    theme(
      plot.title = element_text(
        face = "bold",
        size = 20,        
        hjust = 0.5       
      ),
      plot.subtitle = element_text(
        size = 16,
        hjust = 0.5
      ),
      axis.title = element_text(
        face = "bold",
        size = 18         
      ),
      axis.text = element_text(
        size = 14         
      ),
      panel.grid.minor = element_blank()
    )
)



CONVEXITY_DIR <- "."      

conv_path <- function(instance, suff){
  file.path(CONVEXITY_DIR,
            paste0("convexity_", instance, "_", suff, ".csv"))
}

load_convexity_data <- function(instance) {
  list(
    avg_nodes = read.csv(conv_path(instance, "avg_nodes")),
    avg_edges = read.csv(conv_path(instance, "avg_edges")),
    best_nodes = read.csv(conv_path(instance, "best_nodes")),
    best_edges = read.csv(conv_path(instance, "best_edges")),
    ext_nodes  = read.csv(conv_path(instance, "ext_nodes")),
    ext_edges  = read.csv(conv_path(instance, "ext_edges"))
  )
}

compute_corrs <- function(df, label) {
  df <- df %>% drop_na(obj, sim)
  
  tibble(
    metric   = label,
    n        = nrow(df),
    pearson  = cor(df$obj, df$sim, method = "pearson"),
    spearman = cor(df$obj, df$sim, method = "spearman")
  )
}

summarize_instance_corrs <- function(instance) {
  d <- load_convexity_data(instance)
  
  bind_rows(
    compute_corrs(d$avg_nodes,  "avg_nodes"),
    compute_corrs(d$avg_edges,  "avg_edges"),
    compute_corrs(d$best_nodes, "best_nodes"),
    compute_corrs(d$best_edges, "best_edges"),
    compute_corrs(d$ext_nodes,  "ext_nodes"),
    compute_corrs(d$ext_edges,  "ext_edges")
  ) %>%
    mutate(instance = instance, .before = 1)
}

instances <- c("TSPA", "TSPB")
corr_all <- purrr::map_dfr(instances, summarize_instance_corrs)
corr_all


summarize_similarity <- function(df, instance, metric) {
  df <- df %>% drop_na(sim)
  
  avg_sim <- mean(df$sim)
  min_sim <- min(df$sim)
  max_sim <- max(df$sim)
  
  tibble(
    instance = instance,
    metric   = metric,
    summary  = sprintf("%.2f (%.2f ; %.2f)", avg_sim, min_sim, max_sim)
  )
}

# summary table for one instance
summarize_instance_similarity <- function(instance) {
  d <- load_convexity_data(instance)
  
  bind_rows(
    summarize_similarity(d$avg_nodes,  instance, "avg_nodes"),
    summarize_similarity(d$avg_edges,  instance, "avg_edges"),
    summarize_similarity(d$best_nodes, instance, "best_nodes"),
    summarize_similarity(d$best_edges, instance, "best_edges"),
    summarize_similarity(d$ext_nodes,  instance, "ext_nodes"),
    summarize_similarity(d$ext_edges,  instance, "ext_edges")
  )
}

# Final summary table for both instances
sim_summary_all <- map_dfr(instances, summarize_instance_similarity)
sim_summary_all


get_plot_labels <- function(metric) {
  # Returns a list(title, y) based on metric name
  switch(metric,
         "avg_nodes" = list(
           title = "Objective vs Average similarity – Nodes",
           y     = "Similarity"
         ),
         "avg_edges" = list(
           title = "Objective vs Average similarity – Edges",
           y     = "Similarity"
         ),
         "best_nodes" = list(
           title = "Objective vs Similarity to best found – Nodes",
           y     = "Similarity"
         ),
         "best_edges" = list(
           title = "Objective vs Similarity to best found – Edges",
           y     = "Similarity"
         ),
         "ext_nodes" = list(
           title = "Objective vs Similarity to best existing solution – Nodes",
           y     = "Similarity"
         ),
         "ext_edges" = list(
           title = "Objective vs Similarity to best existing solution – Edges",
           y     = "Similarity"
         ),
         # default fallback
         list(
           title = paste("Objective vs", metric),
           y     = "Similarity"
         )
  )
}


#Plotting

plot_convexity <- function(df, instance, metric, zoom_quant = 0.2) {
  df <- df %>% drop_na(obj, sim)
  
  labels <- get_plot_labels(metric)
  
  p_full <- ggplot(df, aes(x = obj, y = sim)) +
    geom_point(alpha = 0.7, color = '#1f78b4') +
    labs(
      title = paste(instance, "–", labels$title, "(full range)"),
      x = "Objective value",
      y = labels$y
    ) +
    theme_minimal(base_size = 14) +
    theme(
      plot.title = element_text(hjust = 0.5, size = 18, face = "bold"),
      axis.title = element_text(size = 14),
      axis.text  = element_text(size = 12)
    )
  
  # Zoomed: focus on best (lowest objective) solutions
  q_cut <- quantile(df$obj, probs = zoom_quant)
  df_zoom <- df %>% filter(obj <= q_cut)
  
  p_zoom <- ggplot(df_zoom, aes(x = obj, y = sim)) +
    geom_point(alpha = 0.7, color = '#CC0000') +
    labs(
      title = paste(instance, "–", labels$title,
                    sprintf("(best %.0f%% by objective)", zoom_quant * 100)),
      x = "Objective value",
      y = labels$y
    ) +
    theme_minimal(base_size = 14) +
    theme(
      plot.title = element_text(hjust = 0.5, size = 18, face = "bold"),
      axis.title = element_text(size = 14),
      axis.text  = element_text(size = 12)
    )
  
  list(full = p_full, zoom = p_zoom)
}


make_all_plots_for_instance <- function(instance, zoom_quant = 0.2) {
  d <- load_convexity_data(instance)
  
  metrics <- c("avg_nodes", "avg_edges",
               "best_nodes", "best_edges",
               "ext_nodes",  "ext_edges")
  
  for (m in metrics) {
    df <- d[[m]]
    plots <- plot_convexity(df, instance, m, zoom_quant = zoom_quant)
    
    # show in RStudio
    print(plots$full)
    print(plots$zoom)
    
    # save to files
    fname_full <- file.path(CONVEXITY_DIR, paste0(instance, "_", m, "_full.png"))
    fname_zoom <- file.path(CONVEXITY_DIR, paste0(instance, "_", m, "_zoom.png"))
    
    ggsave(fname_full, plots$full + theme(
      plot.title = element_text(size = 40, hjust = 0.5, face = "bold"),
      axis.title = element_text(size = 30),
      axis.text = element_text(size = 30)
    ), width = 7, height = 5, dpi = 300)
    
    ggsave(fname_zoom, plots$zoom + theme(
      plot.title = element_text(size = 20, hjust = 0.5, face = "bold"),
      axis.title = element_text(size = 16),
      axis.text = element_text(size = 14)
    ), width = 7, height = 5, dpi = 300)
    
    
    cat("Saved plots for", instance, m, "->", fname_full, "and", fname_zoom, "\n")
  }
}


make_all_plots_for_instance("TSPA", zoom_quant = 0.2)
make_all_plots_for_instance("TSPB", zoom_quant = 0.2)
