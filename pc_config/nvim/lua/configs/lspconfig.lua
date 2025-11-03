-- load defaults i.e lua_lsp
local nvlsp = require "nvchad.configs.lspconfig"
require("nvchad.configs.lspconfig").defaults()

local lspconfig = require "lspconfig"

-- nvlsp.defaults() -- loads nvchad's defaults


vim.diagnostic.config({ virtual_text = false})


-- EXAMPLE
-- local servers = { "html", "cssls","pylsp", "jedi_language_server"}
-- local servers = { "html", "cssls","pylsp", "jedi_language_server", "clangd"}
--
--
-- require'lspconfig'.pylsp.setup{
--   -- To enable pylsp-rope logging
--   -- cmd = { "pylsp", "-v", "--log-file", "/tmp/nvim-pylsp.log" },
--   cmd = { "pylsp" },
--   on_attach = nvlsp.on_attach,
-- }


-- lsps with default config
-- for _, lsp in ipairs(servers) do
--   lspconfig[lsp].setup {
--     on_attach = nvlsp.on_attach,
--     on_init = nvlsp.on_init,
--     capabilities = nvlsp.capabilities,
--     handlers = {
--       ["textDocument/publishDiagnostics"] = vim.lsp.with(
--         vim.lsp.diagnostic.on_publish_diagnostics, {
--           -- Disable virtual_text
--           virtual_text = false,
--           signs = true,
--           underline = true,
--         }
--       ),
--     }
--   }
-- end

-- configuring single server, example: typescript
-- lspconfig.ts_ls.setup {
--   on_attach = nvlsp.on_attach,
--   on_init = nvlsp.on_init,
--   capabilities = nvlsp.capabilities,
-- }
