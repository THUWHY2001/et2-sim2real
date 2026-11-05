import {cpSync, existsSync, mkdirSync, readdirSync, rmSync, statSync, writeFileSync} from 'node:fs';
import {fileURLToPath} from 'node:url';
import path from 'node:path';

const PUBLIC_EXTENSIONS=new Set(['.html','.css','.js']);
const DEVELOPMENT_DIRECTORIES=['src','scripts','tests'];

export function createWebCopy({root,destination,sourceCommit}){
  const sourceRoot=path.resolve(root);
  const targetRoot=path.resolve(destination);
  const relative=path.relative(sourceRoot,targetRoot);
  if(!relative||relative.startsWith('..')||path.isAbsolute(relative))throw new Error('destination must be a child of root');

  if(existsSync(targetRoot))rmSync(targetRoot,{recursive:true,force:true});
  mkdirSync(targetRoot,{recursive:true});

  for(const entry of readdirSync(sourceRoot)){
    const source=path.join(sourceRoot,entry);
    if(path.resolve(source)===targetRoot||!statSync(source).isFile()||!PUBLIC_EXTENSIONS.has(path.extname(entry)))continue;
    cpSync(source,path.join(targetRoot,entry));
  }
  for(const directory of DEVELOPMENT_DIRECTORIES){
    const source=path.join(sourceRoot,directory);
    if(existsSync(source))cpSync(source,path.join(targetRoot,directory),{recursive:true});
  }

  writeFileSync(path.join(targetRoot,'WEBPAGE_VERSION.md'),`# 网页2\n\n这是网页1在提交 \`${sourceCommit}\` 基础上建立的独立副本。\n\n- 网页1：仓库根目录，默认冻结，不直接修改。\n- 网页2：本目录，后续功能和数据修改在此进行。\n- 网页2的页面、样式、脚本、模型源文件和测试均保存在本目录内。\n`);
}

const invokedPath=process.argv[1]&&path.resolve(process.argv[1]);
if(invokedPath===fileURLToPath(import.meta.url)){
  const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),'..');
  const sourceCommit=process.argv[2];
  if(!sourceCommit)throw new Error('usage: node scripts/create-web-copy.mjs <source-commit>');
  createWebCopy({root,destination:path.join(root,'web2'),sourceCommit});
}
