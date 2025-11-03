return {
  {
    "stevearc/conform.nvim",
    -- event = 'BufWritePre', -- uncomment for format on save
    opts = require "configs.conform",
  },
  {
    "hrsh7th/nvim-cmp",
    lazy = false,
    config = function()
      require "configs.cmp"
    end,
  },
  -- These are some examples, uncomment them if you want to see them work!
  {
    "neovim/nvim-lspconfig",
    lazy = false,
    config = function()
      require "configs.lspconfig"
    end,
  },
  {
    "lervag/vimtex",
    ft = { "tex" },
    -- tag = "v2.15", -- uncomment to pin to a specific release
    init = function()
      -- VimTeX configuration goes here, e.g.
      vim.g.vimtex_view_method = "sioyek"
      vim.g.vimtex_view_sioyek_exe = "sioyek.exe"
      vim.g.vimtex_view_sioyek_options = "--reuse-window --inverse-search 'nvim --headless +%l %f'"
      vim.g.vimtex_complete_commands = true
      vim.g.vimtex_complete_close_braces = true
      vim.g.latex_indent_enabled = 1
      vim.g.tex_comment_nospell = 1
      -- vim.g.vimtex_toc_config = {{"split_pos":"vert rightbelow"}, {"fold_enable":1}, {"layers":['content',],},}
      vim.g.vimtex_quickfix_autoclose_after_keystrokes = 0
      vim.g.vimtex_compiler_method = "latexmk"
      -- vim.g.vimtex_toc_config = {"split_pos":"vert rightbelow","fold_enable":1,"layers":['content']}
      -- require('cmp').setup.buffer { enabled = false }
    end,
  },
  {
    "folke/trouble.nvim",
    opts = {}, -- for default options, refer to the configuration section for custom setup.
    cmd = "Trouble",
    ft = { "python" },
    keys = {
      {
        "<leader>xx",
        "<cmd>Trouble diagnostics toggle<cr>",
        desc = "Diagnostics (Trouble)",
      },
      {
        "<leader>xX",
        "<cmd>Trouble diagnostics toggle filter.buf=0<cr>",
        desc = "Buffer Diagnostics (Trouble)",
      },
      {
        "<leader>cs",
        "<cmd>Trouble symbols toggle focus=false<cr>",
        desc = "Symbols (Trouble)",
      },
      {
        "<leader>cl",
        "<cmd>Trouble lsp toggle focus=false win.position=right<cr>",
        desc = "LSP Definitions / references / ... (Trouble)",
      },
      {
        "<leader>xL",
        "<cmd>Trouble loclist toggle<cr>",
        desc = "Location List (Trouble)",
      },
      {
        "<leader>xQ",
        "<cmd>Trouble qflist toggle<cr>",
        desc = "Quickfix List (Trouble)",
      },
    },
  },
  {
    "dense-analysis/ale",
    lazy = true,
    ft = { "python" },
    config = function()
      -- Configuration goes here.
      local g = vim.g
      g.ale_linters = {
        python = {},
        latex = {},
      }
      g.ale_fixers = {
        python = { "ruff" },
      }
      g.ale_set_highlights = 1
      g.ale_python_pylint_use_global = 1
      g.ale_python_ruff_options = "--line-length=88"
      g.ale_virtualtext_cursor = 0
      g.ale_sign_error = ">>"
      g.ale_sign_warning = "--"
      g.ale_use_neovim_diagnostics_api = 0
    end,
  },
  {
    "nvim-treesitter/nvim-treesitter",
    opts = {
      ensure_installed = {
        "vim",
        "lua",
        "vimdoc",
        "html",
        "css",
        "python",
      },
      highlight = {
        enable = true,
        disable = { "tex", "latex" },
      },
    },
  },
  {
    "chomosuke/typst-preview.nvim",
    lazy = false, -- or ft = 'typst'
    version = "1.*",
    opts = {}, -- lazy.nvim will implicitly calls `setup {}`
  },
  {
    "kaarmu/typst.vim",
    ft = "typst",
    lazy = false,
  },
  {
    "mason-org/mason-lspconfig.nvim",
    lazy = false,
    dependencies = {
      "mason-org/mason.nvim",
      "neovim/nvim-lspconfig",
    },
    opts = {
      automatic_installation = true,
    },
  },
  {
    "mason-org/mason.nvim",
    lazy = false,
    opts = {},
  },
  {
    "github/copilot.vim",
    lazy = false,
    init = function()
      -- VimTeX configuration goes here, e.g.
      vim.g.copilot_workspace_folders = { "~/Projects/scoti", "~/Projects/astrix" }
    end,
  },
  {
    "CopilotC-Nvim/CopilotChat.nvim",
    dependencies = {
      { "github/copilot.vim" }, -- or zbirenbaum/copilot.lua
      { "nvim-lua/plenary.nvim", branch = "master" }, -- for curl, log and async functions
    },
    lazy = false,
    build = "make tiktoken", -- Only on MacOS or Linux
    opts = {
      model = "claude-sonnet-4", -- AI model to use
      temperature = 0.2, -- Lower = focused, higher = creative
      window = {
        layout = "vertical", -- 'vertical', 'horizontal', 'float'
        width = 0.4, -- 50% of screen width
      },
      auto_insert_mode = true, -- Enter insert mode when opening
      mappings = {
        complete = {
          insert = "<S-Tab>",
        },
      },
    },
    -- See Commands section for default commands if you want to lazy load on them
  },
  {
    "nvim-telescope/telescope-ui-select.nvim",
    dependencies = { "nvim-telescope/telescope.nvim" },
    config = function()
      require("telescope").load_extension "ui-select"
    end,
  },
  {
    "stevearc/aerial.nvim",
    lazy = false,
    dependencies = { "nvim-treesitter/nvim-treesitter", "nvim-tree/nvim-web-devicons" },
    cmd = { "AerialToggle", "AerialOpen", "AerialClose", "AerialNavToggle" }, -- lazy-load on demand
    opts = {
      backends = { "lsp", "treesitter", "markdown" },
      layout = { default_direction = "prefer_right", min_width = 16, max_width = { 40, 0.25 } },
      filter_kind = { "Class", "Function", "Method", "Struct", "Interface", "Enum", "Module" }, -- show all kinds
    },
    keys = {
      { "<leader>o", "<cmd>AerialToggle<CR>", desc = "Symbols outline (TOC)" },
    },
  },
}
