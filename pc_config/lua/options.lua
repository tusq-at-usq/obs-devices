require "nvchad.options"

-- add yours here!
local o = vim.o
local v = vim
o.foldmethod = 'expr'
o.foldexpr = 'nvim_treesitter#foldexpr()'

o.clipboard = ""

v.opt.foldenable = false

vim.opt.clipboard = "unnamedplus"   -- set
vim.opt.clipboard:append("unnamed") -- add more, safe with lists
vim.opt.number = true

local o = vim.o
o.cursorlineopt ='both' -- to enable cursorline!

-- Disable wrapping only for markdown and text
vim.api.nvim_create_autocmd("FileType", {
  pattern = { "markdown", "tex", "text", "latex"},
  callback = function()
    vim.opt_local.breakindent = true
    vim.opt_local.linebreak = true
  end,
})

